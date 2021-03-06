#!/usr/bin/env python3
import numpy as np
import carla
import rospy
import sys
from graic_msgs.msg import LaneInfo
from graic_msgs.msg import LaneList
from graic_msgs.msg import ObstacleInfo
from graic_msgs.msg import ObstacleList
from graic_msgs.msg import BBSingleInfo
from geometry_msgs.msg import Vector3
class PerceptionModule():
    def __init__(self, carla_world, role_name, radius=15):
        self.sensing_radius = radius # default ?????
        self.world = carla_world
        self.vehicle = None
        self.role_name = role_name
        self.find_ego_vehicle()
        
    # find ego vehicle
    def find_ego_vehicle(self):
        for actor in self.world.get_actors():
            if actor.attributes.get('role_name') == self.role_name:
            # if 'vehicle' in actor.type_id:
                self.vehicle = actor
                break
    # return all the obstacles within the sensing radius of the vehicle
    def get_all_obstacles_within_range(self):
        # get every actor on stage
        if self.vehicle == None:
            self.find_ego_vehicle()
            # rospy.loginfo("No ego vehicle.")
            return
        vehicle = self.vehicle
        vehicle_loc = vehicle.get_location()
        all_actors = self.world.get_actors()
        radius = self.sensing_radius
        filtered_obstacles = []
        for actor in all_actors:
            # get actor's location
            cur_loc = actor.get_location()
            # determine whether actor is within the radius
            if vehicle_loc.distance(cur_loc) <= radius:
                # we need to throw out actors such as camera
                # types we need: vehicle, walkers, Traffic signs and traffic lights
                # reference: https://github.com/carla-simulator/carla/blob/master/PythonAPI/carla/scene_layout.py

                if 'vehicle' in actor.type_id and actor.id != vehicle.id:
                    filtered_obstacles.append(actor)
                elif 'pedestrian' in actor.type_id:
                    filtered_obstacles.append(actor)
                elif 'static.prop' in actor.type_id:
                    filtered_obstacles.append(actor)
        return filtered_obstacles

    def set_radius(self, new_radius):
        self.sensing_radius = new_radius
    
    def get_radius(self):
        return self.sensing_radius
    # determine if the bounding box contains the given point
    def boundingbox_within_range(self, box, self_loc):
        radius = self.sensing_radius
        box_loc = box.location
        # cast vertices to the same plane as the ego vehicle
        x_half_width = box.extent.x
        y_half_width = box.extent.y
        v1 = carla.Location(box_loc.x+x_half_width, box_loc.y+y_half_width, self_loc.z)
        v2 = carla.Location(box_loc.x+x_half_width, box_loc.y-y_half_width, self_loc.z)
        v3 = carla.Location(box_loc.x-x_half_width, box_loc.y+y_half_width, self_loc.z)
        v4 = carla.Location(box_loc.x-x_half_width, box_loc.y-y_half_width, self_loc.z)
        if self_loc.y <= v1.y and self_loc.y >= v2.y:
            if (self.distance_from_point_to_line(self_loc, v1, v2) <= radius):
                return True
            if (self.distance_from_point_to_line(self_loc, v3, v4) <= radius):
                return True
        if self_loc.x <= v1.x and self_loc.x >= v3.x:
            if (self.distance_from_point_to_line(self_loc, v1, v3) <= radius):
                return True
            if (self.distance_from_point_to_line(self_loc, v2, v4) <= radius):
                return True
        # calculate distance to vertices
        vertices = [v1, v2, v3, v4]
        for v in vertices:
            if self.distance_between_points(self_loc ,v) <= radius:
                return True
        return False
        # calculate the distance from point x to the line connecting p1 and p2
    def distance_from_point_to_line(self, x, p1, p2):
        num = np.abs((p2.x - p1.x)*(p1.y - x.y) - (p1.x - x.x)*(p2.y - p1.y))
        denom = np.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)
        return num/denom
    def distance_between_points(self, p1, p2):
        return np.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)
    # get city objects in terms of info on their bounding box
    # return:vertices location in world space
    # side effect: draw bounding box
    def get_bb_global_ver_within_range(self, obj_type):
        # TODO: need to check validity of object type
        if self.vehicle == None:
            self.find_ego_vehicle()
            # rospy.loginfo("No ego vehicle.")
            return
        all_env_bbs = self.world.get_environment_objects(obj_type)
        vehicle = self.vehicle
        self_loc = vehicle.get_location()
        radius = self.sensing_radius
        filtered_obstacles = []
        for env_bb in all_env_bbs:
            # center of the bounding box in world space
            center_of_box = env_bb.bounding_box.location
            # dist = np.sqrt((center_of_box.x - self_loc.x)**2 + (center_of_box.y-self_loc.y)**2 + (center_of_box.z-self_loc.z)**2)
            dist = self.distance_between_points(center_of_box, self_loc)
            if dist <= radius:
                filtered_obstacles.append((env_bb.bounding_box.get_local_vertices(), env_bb.name, env_bb.id, env_bb.transform.location, env_bb.bounding_box))
            elif self.boundingbox_within_range(env_bb.bounding_box, self_loc):
                filtered_obstacles.append((env_bb.bounding_box.get_local_vertices(), env_bb.name, env_bb.id, env_bb.transform.location, env_bb.bounding_box))
        return filtered_obstacles
    # get set of waypoints separated by parameter -- distance -- along the lane
    # reference: https://github.com/carla-simulator/carla/issues/1254
    def get_lane_markers(self, distance=0.5):
        if self.vehicle == None:
            self.find_ego_vehicle()
            # rospy.loginfo("No ego vehicle.")
            return
        vehicle = self.vehicle
        carla_map = self.world.get_map()
        vehicle_location = vehicle.get_location()
        # get a nearest waypoint
        cur_waypoint = carla_map.get_waypoint(vehicle_location)
        # return list of waypoints from cur_waypoint to 10 meters ahead
        waypoints = cur_waypoint.next_until_lane_end(distance)
        num_of_wp = len(waypoints)
        if num_of_wp > 20:
            waypoints = waypoints[0:20]
        if num_of_wp < 20:
            if waypoints[num_of_wp-1].is_junction:
                # debug
                waypoints.append(waypoints[num_of_wp-1].get_junction().get_waypoints(lane_type=carla.LaneType.Driving))
                # approximate a point in next lane
                vec = vehicle.get_velocity()
                vec = vec_end/np.sqrt((vec.x)**2 + (vec.y)**2 + (vec.z)**2)
                vec = vec*2
                vec_end = waypoints[num_of_wp-1].transform.location
                if num_of_wp > 1:
                    vec_start = waypoints[num_of_wp-2].transform.location
                    vec = vec_end - vec_start
                    vec = vec*5
                approx_next_loc = vec + vec_end
                approx_waypoint = carla_map.get_waypoint(approx_next_loc)
                markers_before = approx_waypoint.previous_until_lane_start(distance)
                waypoints.append(markers_before)
                new_num_of_markers = len(waypoints)
                if new_num_of_markers < 20:
                    markers_after = approx_waypoint.next_until_lane_end(distance)
                    waypoints.append(markers_after)
                    if len(waypoints) > 20:
                            waypoints = waypoints[0:20]
        return waypoints
# helper function for calculating lane markers
# approximate the locations of lane markers by the assumptions: 
# 1. the lane markers have the same z coordinates as the current location 
# 2. the vector pointing from cur_loc to any one of the lane marker is perpendicular to the vector pointing from cur_loc to the next way point ahead
def get_markers(cur_loc, v, w):
    x = v.x
    y = v.y
    z = v.z
    m1 = np.sqrt((w*w*y*y)/4/(x*x+y*y))
    n1 = (-x/y)*m1
    m2 = -m1
    n2 = (-x/y)*m2
    marker1 = cur_loc + carla.Location(m1, n1, 0)
    marker2 = cur_loc + carla.Location(m2, n2, 0)
    x1 = np.array([marker1.x, marker1.y, marker1.z])
    x2 = np.array([x, y, z])
    if np.cross(x1, x2)[2] > 0:
        return (marker1, marker2)
    else:
        return (marker2, marker1)
# helper function for creating lane markers information
def get_marker_info(loc, rot):
    mark_loc = Vector3()
    mark_rot = Vector3()
    mark_loc.x = loc.x
    mark_loc.y = loc.y
    mark_loc.z = loc.z
    mark_rot.x = rot.pitch
    mark_rot.y = rot.yaw
    mark_rot.z = rot.roll
    return (mark_loc, mark_rot)
# publish obstacles and lane waypoints information
def publisher(percep_mod, role_name, label_list):
    # main function
    obs_pub = rospy.Publisher('/carla/%s/obstacles'%role_name, ObstacleList, queue_size=1)
    lane_pub = rospy.Publisher('/carla/%s/lane_markers'%role_name, LaneInfo, queue_size=1)
    rate = rospy.Rate(20)
    draw_counter = 0
    while not rospy.is_shutdown():
        obs = percep_mod.get_all_obstacles_within_range()
        lp = percep_mod.get_lane_markers()
        obs_msg = []
        # mark_msg_center = []
        # mark_msg_left = []
        # mark_msg_right = []
        draw_counter += 1
        if obs is None or lp is None:
            continue
        for ob in obs:
            info = ObstacleInfo()
            # fill the fields for current obstacle
            info.obstacle_name = ob.type_id
            info.obstacle_id = ob.id
            loc = ob.get_location()
            info.location.x = loc.x
            info.location.y = loc.y
            info.location.z = loc.z
            # draw bounding box for vehicles and walkers
            if 'vehicle' in ob.type_id or 'pedestrian' in ob.type_id:
                bb = ob.bounding_box
                bb.location = loc
                bb.rotation = ob.get_transform().rotation
                if draw_counter % 8 == 0:
                    percep_mod.world.debug.draw_box(bb, bb.rotation, color=carla.Color(255, 0, 0), life_time=1)
                vertices = bb.get_local_vertices()
                for v in vertices:
                    vertex = BBSingleInfo()
                    vertex.vertex_location.x = v.x
                    vertex.vertex_location.y = v.y
                    vertex.vertex_location.z = v.z
                    info.vertices_locations.append(vertex)
            obs_msg.append(info)
        # TODO: fix left and right markers' rotation
        # TODO: add gradient
        marker_msg = LaneInfo()
        marker_center_list = LaneList()
        marker_left_list = LaneList()
        marker_right_list = LaneList()
        for i in range(len(lp)):
            p = lp[i]
            trans = p.transform
            loc = trans.location
            rot  = trans.rotation
            mark_loc, mark_rot = get_marker_info(loc, rot)
            marker_msg.lane_state = abs(p.lane_id)
            marker_center_list.location.append(mark_loc)
            marker_center_list.rotation.append(mark_rot)
            # draw left and right lane markers
            width = p.lane_width
            next_p = None
            if i != len(lp) - 1:
                next_p = lp[i+1]
                vec = next_p.transform.location - loc
                mark1 = carla.Location(loc.x-width/2, loc.y,loc.z)
                mark2 = carla.Location(loc.x+width/2, loc.y,loc.z)
                if vec.y == 0:
                    mark_loc, mark_rot = get_marker_info(mark1, rot)
                    marker_left_list.location.append(mark_loc)
                    marker_left_list.rotation.append(mark_rot)
                    mark_loc, mark_rot = get_marker_info(mark2, rot)
                    marker_right_list.location.append(mark_loc)
                    marker_right_list.rotation.append(mark_rot)
                    percep_mod.world.debug.draw_point(mark1,life_time=1)
                    percep_mod.world.debug.draw_point(mark2,life_time=1)
                else:    
                    mark1, mark2 = get_markers(loc, vec, width)
                    mark_loc, mark_rot = get_marker_info(mark1, rot)
                    marker_left_list.location.append(mark_loc)
                    marker_left_list.rotation.append(mark_rot)
                    mark_loc, mark_rot = get_marker_info(mark2, rot)
                    marker_right_list.location.append(mark_loc)
                    marker_right_list.rotation.append(mark_rot)
                    percep_mod.world.debug.draw_point(mark1,life_time=1)
                    percep_mod.world.debug.draw_point(mark2,life_time=1)
            percep_mod.world.debug.draw_point(trans.location,life_time=1)
        marker_msg.lane_markers_center = marker_center_list
        marker_msg.lane_markers_left = marker_left_list
        marker_msg.lane_markers_right = marker_right_list
        for label in label_list:
            # get all vertices of all bounding boxes which are within the radius with label 'label'
            vertices_of_cur_label = percep_mod.get_bb_global_ver_within_range(label)
            if not vertices_of_cur_label:
                continue
            for vertices_of_one_box in vertices_of_cur_label:
                info = ObstacleInfo()
                for loc in vertices_of_one_box[0]:
                    vertex = BBSingleInfo()
                    vertex.vertex_location.x = loc.x
                    vertex.vertex_location.y = loc.y
                    vertex.vertex_location.z = loc.z
                    info.vertices_locations.append(vertex)
                info.obstacle_name = str(vertices_of_one_box[1])
                info.obstacle_id = vertices_of_one_box[2] % 1013 # NOTE 1013 is a magic number, the purpose is to shorten the id from a very large number
                obstacle_loc = vertices_of_one_box[3]
                info.location.x = obstacle_loc.x
                info.location.y = obstacle_loc.y
                info.location.z = obstacle_loc.z
                bb = vertices_of_one_box[4]
                percep_mod.world.debug.draw_box(bb, bb.rotation, life_time=1)
                obs_msg.append(info)
        obs_pub.publish(obs_msg)
        lane_pub.publish(marker_msg)
        rate.sleep()


if __name__ == "__main__":
    # reference: https://github.com/SIlvaMFPedro/ros_bridge/blob/master/carla_waypoint_publisher/src/carla_waypoint_publisher/carla_waypoint_publisher.py
    rospy.init_node('graic_raceinfo_publisher', anonymous=True)
    # host = rospy.get_param("/carla/host", "127.0.0.1")
    # port = rospy.get_param("/carla/host", 2000)
    # timeout = rospy.get_param("/carla/timeout", 10)
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    client = carla.Client('localhost', 2000)
    # client.set_timeout(timeout)
    world = client.get_world()
    pm = PerceptionModule(world, role_name)
    default_list = [carla.CityObjectLabel.Buildings, carla.CityObjectLabel.Fences, carla.CityObjectLabel.Sidewalks, carla.CityObjectLabel.Walls, carla.CityObjectLabel.Vegetation]
    try:
        publisher(pm, role_name, default_list)
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down raceinfo publisher")
