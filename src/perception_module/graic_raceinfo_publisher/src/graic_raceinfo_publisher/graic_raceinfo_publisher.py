#!/usr/bin/env python3
from operator import le
from pickle import NONE, TRUE
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
    def __init__(self, carla_world, role_name, radius=60):
        self.sensing_radius = radius # default ?????
        self.world = carla_world
        self.vehicle = None
        self.role_name = role_name
        self.find_ego_vehicle()
    
    def get_vehicle_location(self):
        vehicle = self.vehicle
        vehicle_loc = vehicle.get_location()
        return vehicle_loc
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
                # we need to exclude actors such as camera
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
    def get_lane_markers(self, distance=0.5, num_of_points=20):
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
        waypoints = []
        for i in range(num_of_points):
            waypoints.extend(cur_waypoint.next((i+1)*distance))
        return waypoints
    
    def get_left_lane_markers(self, distance=0.5, num_of_points=20):
        if self.vehicle == None:
            self.find_ego_vehicle()
            return
        vehicle = self.vehicle
        carla_map = self.world.get_map()
        vehicle_location = vehicle.get_location()
        cur_waypoint = carla_map.get_waypoint(vehicle_location)
        left_waypoint = cur_waypoint.get_left_lane()
        if left_waypoint is None:
            return None
        waypoints = []
        for i in range(num_of_points):
            waypoints.extend(left_waypoint.next((i+1)*distance))
        return waypoints
    def get_right_lane_markers(self, distance=0.5, num_of_points=20):
        if self.vehicle == None:
            self.find_ego_vehicle()
            return
        vehicle = self.vehicle
        carla_map = self.world.get_map()
        vehicle_location = vehicle.get_location()
        cur_waypoint = carla_map.get_waypoint(vehicle_location)
        right_waypoint = cur_waypoint.get_right_lane()
        if right_waypoint is None:
            return None
        waypoints = []
        for i in range(num_of_points):
            waypoints.extend(right_waypoint.next((i+1)*distance))
        return waypoints
    def get_left_boundary_lane_center_markers(self, distance=0.5, num_of_points=20):
        cur_loc = self.get_vehicle_location()
        carla_map = self.world.get_map()
        cur_waypoint = carla_map.get_waypoint(cur_loc)
        left_waypoint_tmp = cur_waypoint.get_left_lane()
        left_waypoint = None
        while left_waypoint_tmp is not None:
            left_waypoint = left_waypoint_tmp
            left_waypoint_tmp = left_waypoint_tmp.get_left_lane()
        if left_waypoint is None:
            return None
        waypoints = []
        for i in range(num_of_points):
            waypoints.extend(left_waypoint.next((i+1)*distance))
        return waypoints
    def get_right_boundary_lane_center_markers(self, distance=0.5, num_of_points=20):
        cur_loc = self.get_vehicle_location()
        carla_map = self.world.get_map()
        cur_waypoint = carla_map.get_waypoint(cur_loc)
        right_waypoint_tmp = cur_waypoint.get_left_lane()
        right_waypoint = None
        while right_waypoint_tmp is not None:
            right_waypoint = right_waypoint_tmp
            right_waypoint_tmp = right_waypoint_tmp.get_right_lane()
        if right_waypoint is None:
            return None
        waypoints = []
        for i in range(num_of_points):
            waypoints.extend(right_waypoint.next((i+1)*distance))
        return waypoints
def get_right_marker(cur_transform, width):
    vec = cur_transform.get_right_vector()
    vec_normalized = vec/np.sqrt((vec.x)**2 + (vec.y)**2 + (vec.z)**2)
    cur_loc = cur_transform.location
    approx_right_marker = cur_loc + (width/2)*vec_normalized
    return carla.Location(approx_right_marker.x, approx_right_marker.y, approx_right_marker.z)
def get_left_marker(cur_transform, width):
    vec = cur_transform.get_right_vector()
    vec_normalized = vec/np.sqrt((vec.x)**2 + (vec.y)**2 + (vec.z)**2)
    cur_loc = cur_transform.location
    approx_left_marker = cur_loc - (width/2)*vec_normalized
    return carla.Location(approx_left_marker.x, approx_left_marker.y, approx_left_marker.z)
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
def fill_marker_msg(lp, percep_mod, visualize=True, side=0):
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
        if side == 0:
            percep_mod.world.debug.draw_point(loc, life_time=1)
        # draw left and right lane markers
        left_marker_loc = get_left_marker(trans, p.lane_width)
        left_marker_loc_info, left_marker_rot_info = get_marker_info(left_marker_loc, rot)
        marker_left_list.location.append(left_marker_loc_info)
        # TODO: How to approximate rotation???
        marker_left_list.rotation.append(left_marker_rot_info)
        if side == 1 or side == 0:
            percep_mod.world.debug.draw_point(left_marker_loc, life_time=1)
        right_marker_loc = get_right_marker(trans, p.lane_width)
        right_marker_loc_info, right_marker_rot_info = get_marker_info(right_marker_loc, rot) 
        marker_right_list.location.append(right_marker_loc_info)
        marker_right_list.rotation.append(right_marker_rot_info)
        if side == -1 or side == 0:
            percep_mod.world.debug.draw_point(right_marker_loc, life_time=1)
    marker_msg.lane_markers_center = marker_center_list
    marker_msg.lane_markers_left = marker_left_list
    marker_msg.lane_markers_right = marker_right_list
    return marker_msg
# publish obstacles and lane waypoints information
def publisher(percep_mod, role_name, label_list):
    # main function
    obs_pub = rospy.Publisher('/carla/%s/obstacles'%role_name, ObstacleList, queue_size=1)
    lane_pub = rospy.Publisher('/carla/%s/lane_markers'%role_name, LaneInfo, queue_size=1)
    left_lane_pub = rospy.Publisher('/carla/%s/left_lane_markers'%role_name, LaneList, queue_size=1)
    right_lane_pub = rospy.Publisher('/carla/%s/right_lane_markers'%role_name, LaneList, queue_size=1)
    rate = rospy.Rate(20)
    draw_counter = 0
    while not rospy.is_shutdown():
        obs = percep_mod.get_all_obstacles_within_range()
        lane_markers = percep_mod.get_lane_markers()
        obs_msg = []
        draw_counter += 1
        if obs is None or lane_markers is None:
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
        marker_msg = fill_marker_msg(lane_markers, percep_mod)
        left_boundary_lane_center_markers = percep_mod.get_left_boundary_lane_center_markers()
        if left_boundary_lane_center_markers is None:
            left_boundary_lane_center_markers = lane_markers
        right_boundary_lane_center_markers = percep_mod.get_right_boundary_lane_center_markers()
        if right_boundary_lane_center_markers is None:
            right_boundary_lane_center_markers = lane_markers
        left_marker_info = fill_marker_msg(left_boundary_lane_center_markers, percep_mod, side=-1)
        right_marker_info = fill_marker_msg(right_boundary_lane_center_markers, percep_mod, side=1)
        # TODO: sidewalks also return lane markers???
        left_marker_msg = left_marker_info.lane_markers_right
        right_marker_msg = right_marker_info.lane_markers_left 
        # NOTE Environment objects note used now    
        # for label in label_list:
        #     # get all vertices of all bounding boxes which are within the radius with label 'label'
        #     vertices_of_cur_label = percep_mod.get_bb_global_ver_within_range(label)
        #     if not vertices_of_cur_label:
        #         continue
        #     for vertices_of_one_box in vertices_of_cur_label:
        #         info = ObstacleInfo()
        #         for loc in vertices_of_one_box[0]:
        #             vertex = BBSingleInfo()
        #             vertex.vertex_location.x = loc.x
        #             vertex.vertex_location.y = loc.yospy.Publisher('/carla/%s/lane_markers'%role_name, LaneList, queue_size=1)
        #             vertex.vertex_location.z = loc.z
        #             info.vertices_locations.append(vertex)
        #         info.obstacle_name = str(vertices_of_one_box[1])
        #         info.obstacle_id = vertices_of_one_box[2] % 1013 # NOTE 1013 is a magic number, the purpose is to shorten the id from a very large number
        #         obstacle_loc = vertices_of_one_box[3]
        #         info.location.x = obstacle_loc.x
        #         info.location.y = obstacle_loc.y
        #         info.location.z = obstacle_loc.z
        #         bb = vertices_of_one_box[4]
        #         percep_mod.world.debug.draw_box(bb, bb.rotation, life_time=1)
        #         obs_msg.append(info)
        obs_pub.publish(obs_msg)
        lane_pub.publish(marker_msg)
        left_lane_pub.publish(left_marker_msg)
        right_lane_pub.publish(right_marker_msg)
        rate.sleep()


if __name__ == "__main__":
    # reference: https://github.com/SIlvaMFPedro/ros_bridge/blob/master/carla_waypoint_publisher/src/carla_waypoint_publisher/carla_waypoint_publisher.py
    rospy.init_node('graic_raceinfo_publisher', anonymous=True)
    host = rospy.get_param("~host", "localhost")
    port = rospy.get_param("~port", 2000)
    # timeout = rospy.get_param("/carla/timeout", 10)
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    client = carla.Client(host, port)
    # client.set_timeout(timeout)
    world = client.get_world()
    pm = PerceptionModule(world, role_name)
    default_list = [carla.CityObjectLabel.Buildings, carla.CityObjectLabel.Fences, carla.CityObjectLabel.Sidewalks, carla.CityObjectLabel.Walls, carla.CityObjectLabel.Vegetation]
    try:
        publisher(pm, role_name, default_list)
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down raceinfo publisher")