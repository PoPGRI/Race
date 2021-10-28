#!/usr/bin/env python3
from operator import le
from pickle import NONE, TRUE
import numpy as np
import carla
import rospy
import sys
from graic_msgs.msg import LocationInfo
from graic_msgs.msg import LaneInfo
from graic_msgs.msg import LaneList
from graic_msgs.msg import ObstacleInfo
from graic_msgs.msg import ObstacleList
from graic_msgs.msg import BBSingleInfo
from geometry_msgs.msg import Vector3


class PerceptionModule():
    def __init__(self, carla_world, role_name, radius=60):
        self.sensing_radius = radius  # default ?????
        self.world = carla_world
        self.vehicle = None
        self.role_name = role_name
        self.find_ego_vehicle()
        self.carla_map = self.world.get_map()

    def get_vehicle_location(self):
        return self.vehicle.get_location()

    def get_vehicle_rotation(self):
        return self.vehicle.get_transform().rotation

    def get_vehicle_velocity(self):
        return self.vehicle.get_velocity()

    # find ego vehicle
    def find_ego_vehicle(self):
        for actor in self.world.get_actors():
            if actor.attributes.get('role_name') == self.role_name:
                self.vehicle = actor
                break

    def getName(self):
        return self.vehicle.type_id

    def getId(self):
        return self.vehicle.id

    # return all the obstacles within the sensing radius of the vehicle
    def get_all_obstacles_within_range(self):
        # get every actor on stage
        vehicle_loc = self.get_vehicle_location()
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
                if 'vehicle' in actor.type_id and actor.id != self.vehicle.id:
                    filtered_obstacles.append(actor)
                elif 'pedestrian' in actor.type_id:
                    filtered_obstacles.append(actor)
                elif 'static.prop' in actor.type_id:
                    filtered_obstacles.append(actor)
        return filtered_obstacles

    # get set of waypoints separated by parameter -- distance -- along the lane
    def get_lane_markers(self, distance=0.5, num_of_points=20):
        carla_map = self.carla_map
        vehicle_location = self.get_vehicle_location()
        # get a nearest waypoint
        cur_waypoint = carla_map.get_waypoint(vehicle_location)
        # return list of waypoints from cur_waypoint to 10 meters ahead
        waypoints = []
        for i in range(num_of_points):
            waypoints.extend(cur_waypoint.next((i + 1) * distance))
        return waypoints

    def get_left_boundary_lane_center_markers(self,
                                              distance=0.5,
                                              num_of_points=20):
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
            waypoints.extend(left_waypoint.next((i + 1) * distance))
        return waypoints

    def get_right_boundary_lane_center_markers(self,
                                               distance=0.5,
                                               num_of_points=20):
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
            waypoints.extend(right_waypoint.next((i + 1) * distance))
        return waypoints


def get_right_marker(cur_transform, width):
    vec = cur_transform.get_right_vector()
    vec_normalized = vec / np.sqrt((vec.x)**2 + (vec.y)**2 + (vec.z)**2)
    cur_loc = cur_transform.location
    approx_right_marker = cur_loc + (width / 2) * vec_normalized
    return carla.Location(approx_right_marker.x, approx_right_marker.y,
                          approx_right_marker.z)


def get_left_marker(cur_transform, width):
    vec = cur_transform.get_right_vector()
    vec_normalized = vec / np.sqrt((vec.x)**2 + (vec.y)**2 + (vec.z)**2)
    cur_loc = cur_transform.location
    approx_left_marker = cur_loc - (width / 2) * vec_normalized
    return carla.Location(approx_left_marker.x, approx_left_marker.y,
                          approx_left_marker.z)


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
        rot = trans.rotation
        mark_loc, mark_rot = get_marker_info(loc, rot)
        marker_msg.lane_state = abs(p.lane_id)
        marker_center_list.location.append(mark_loc)
        marker_center_list.rotation.append(mark_rot)
        if side == 0:
            percep_mod.world.debug.draw_point(loc, life_time=1)
        # draw left and right lane markers
        left_marker_loc = get_left_marker(trans, p.lane_width)
        left_marker_loc_info, left_marker_rot_info = get_marker_info(
            left_marker_loc, rot)
        marker_left_list.location.append(left_marker_loc_info)
        # TODO: How to approximate rotation???
        marker_left_list.rotation.append(left_marker_rot_info)
        if side == 1 or side == 0:
            percep_mod.world.debug.draw_point(left_marker_loc, life_time=1)
        right_marker_loc = get_right_marker(trans, p.lane_width)
        right_marker_loc_info, right_marker_rot_info = get_marker_info(
            right_marker_loc, rot)
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
    loc_pub = rospy.Publisher('/carla/%s/location' % role_name,
                              LocationInfo,
                              queue_size=1)
    obs_pub = rospy.Publisher('/carla/%s/obstacles' % role_name,
                              ObstacleList,
                              queue_size=1)
    lane_pub = rospy.Publisher('/carla/%s/lane_markers' % role_name,
                               LaneInfo,
                               queue_size=1)
    left_lane_pub = rospy.Publisher('/carla/%s/left_lane_markers' % role_name,
                                    LaneList,
                                    queue_size=1)
    right_lane_pub = rospy.Publisher('/carla/%s/right_lane_markers' %
                                     role_name,
                                     LaneList,
                                     queue_size=1)
    draw_counter = 0
    prev_location = None

    while not rospy.is_shutdown():
        print('alive')
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
                    percep_mod.world.debug.draw_box(bb,
                                                    bb.rotation,
                                                    color=carla.Color(
                                                        255, 0, 0),
                                                    life_time=1)
                vertices = bb.get_local_vertices()
                for v in vertices:
                    vertex = BBSingleInfo()
                    vertex.vertex_location.x = v.x
                    vertex.vertex_location.y = v.y
                    vertex.vertex_location.z = v.z
                    info.vertices_locations.append(vertex)
            obs_msg.append(info)
        marker_msg = fill_marker_msg(lane_markers, percep_mod)
        left_boundary_lane_center_markers = percep_mod.get_left_boundary_lane_center_markers(
        )
        if left_boundary_lane_center_markers is None:
            left_boundary_lane_center_markers = lane_markers
        right_boundary_lane_center_markers = percep_mod.get_right_boundary_lane_center_markers(
        )
        if right_boundary_lane_center_markers is None:
            right_boundary_lane_center_markers = lane_markers
        left_marker_info = fill_marker_msg(left_boundary_lane_center_markers,
                                           percep_mod,
                                           side=-1)
        right_marker_info = fill_marker_msg(right_boundary_lane_center_markers,
                                            percep_mod,
                                            side=1)
        # TODO: sidewalks also return lane markers???
        left_marker_msg = left_marker_info.lane_markers_right
        right_marker_msg = right_marker_info.lane_markers_left
        obs_pub.publish(obs_msg)
        lane_pub.publish(marker_msg)
        left_lane_pub.publish(left_marker_msg)
        right_lane_pub.publish(right_marker_msg)

        loc_info = LocationInfo()
        loc_info.actor_name = percep_mod.getName()
        loc_info.actor_id = percep_mod.getId()
        location = percep_mod.get_vehicle_location()
        if prev_location is None:
            prev_location = location
        loc_info.location.x = location.x
        loc_info.location.y = location.y
        loc_info.location.z = location.z
        rotation = percep_mod.get_vehicle_rotation()
        loc_info.rotation.x = rotation.roll
        loc_info.rotation.y = rotation.pitch
        loc_info.rotation.z = rotation.yaw
        velocity = percep_mod.get_vehicle_velocity()
        loc_info.velocity.x = velocity.x
        loc_info.velocity.y = velocity.y
        loc_info.velocity.z = velocity.z
        if (loc_info.velocity.x == loc_info.velocity.y == loc_info.velocity.z
                == 0):
            loc_info.velocity.x = (location.x - prev_location.x) * 20.  # FIXME
            loc_info.velocity.y = (location.y - prev_location.y) * 20.  # FIXME
        prev_location = location
        loc_pub.publish(loc_info)
        percep_mod.world.wait_for_tick(seconds=1000)


if __name__ == "__main__":
    # reference: https://github.com/SIlvaMFPedro/ros_bridge/blob/master/carla_waypoint_publisher/src/carla_waypoint_publisher/carla_waypoint_publisher.py
    rospy.init_node('graic_perception', anonymous=True)
    host = rospy.get_param("~host", "localhost")
    port = rospy.get_param("~port", 2000)
    # timeout = rospy.get_param("/carla/timeout", 10)
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    client = carla.Client(host, port)
    # client.set_timeout(timeout)
    world = client.get_world()
    pm = PerceptionModule(world, role_name)
    pm.world.wait_for_tick(seconds=1000)

    while not pm.vehicle:
        pm.find_ego_vehicle()

    default_list = [
        carla.CityObjectLabel.Buildings, carla.CityObjectLabel.Fences,
        carla.CityObjectLabel.Sidewalks, carla.CityObjectLabel.Walls,
        carla.CityObjectLabel.Vegetation
    ]
    try:
        publisher(pm, role_name, default_list)
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down raceinfo publisher")
