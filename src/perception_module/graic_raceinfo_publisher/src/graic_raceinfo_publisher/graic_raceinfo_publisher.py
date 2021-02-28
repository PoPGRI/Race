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
class PerceptionModule():
    def __init__(self, carla_world, role_name, radius=15):
        self.sensing_radius = radius # default ?????
        self.world = carla_world
        self.vehicle = None
        self.role_name = role_name
        
        self.find_ego_vehicle()
        
    # find ego vehicle
    # reference: 
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
    
    # get set of waypoints separated by parameter -- distance -- along the lane
    # reference: https://github.com/carla-simulator/carla/issues/1254
    def get_lane_way_point(self, distance=0.5):
        if self.vehicle == None:
            self.find_ego_vehicle()
            # rospy.loginfo("No ego vehicle.")
            return
        vehicle = self.vehicle
        carla_map = self.world.get_map()
        vehicle_location = vehicle.get_location()
        # get a nearest waypoint
        cur_waypoint = carla_map.get_waypoint(vehicle_location)
        # rospy.logwarn("Current road id is %s"%cur_waypoint.road_id)
        # rospy.logwarn("Current section id is %s"%cur_waypoint.section_id)
        # rospy.logwarn("Current lane id is %s"%cur_waypoint.lane_id)
        # return list of waypoints from cur_waypoint to 10 meters ahead
        wp_to_end = cur_waypoint.next_until_lane_end(distance)
        if len(wp_to_end) > 20:
            wp_to_end = wp_to_end[0:20]
        return wp_to_end
# calculate distance between p1 and p2
def distance_between_points(p1, p2):
        return np.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)
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
    return (marker1, marker2)
# publish obstacles and lane waypoints information
def publisher(percep_mod, role_name):
    # main function
    obs_pub = rospy.Publisher('/carla/%s/obstacles'%role_name, ObstacleList, queue_size=1)
    lane_pub = rospy.Publisher('/carla/%s/lane_waypoints'%role_name, LaneList, queue_size=1)
    rate = rospy.Rate(20)
    draw_counter = 0
    while not rospy.is_shutdown():
        obs = percep_mod.get_all_obstacles_within_range()
        lp = percep_mod.get_lane_way_point()
        obsmsg = []
        lpmsg = []
        draw_counter += 1
        if obs is None or lp is None:
            continue
        for ob in obs:
            temp = ObstacleInfo()
            # fill the fields for current obstacle
            temp.obstacle_name = ob.type_id
            temp.obstacle_id = ob.id
            loc = ob.get_location()
            temp.location.x = loc.x
            temp.location.y = loc.y
            temp.location.z = loc.z
            # draw bounding box for vehicles and walkers
            if 'vehicle' in ob.type_id or 'pedestrian' in ob.type_id:
                bb = ob.bounding_box
                bb.location = loc
                bb.rotation = ob.get_transform().rotation
                if draw_counter % 8 == 0:
                    percep_mod.world.debug.draw_box(bb, bb.rotation, color=carla.Color(255, 0, 0), life_time=0.5)
                vertices = bb.get_local_vertices()
                for v in vertices:
                    vertex = BBSingleInfo()
                    vertex.vertex_location.x = v.x
                    vertex.vertex_location.y = v.y
                    vertex.vertex_location.z = v.z
                    temp.vertices_locations.append(vertex)
            obsmsg.append(temp)
        for i in range(len(lp)):
            p = lp[i]
            temp = LaneInfo()
            trans = p.transform
            loc = trans.location
            rot  = trans.rotation
            temp.location.x = loc.x
            temp.location.y = loc.y
            temp.location.z = loc.z
            temp.rotation.x = rot.pitch
            temp.rotation.y = rot.yaw
            temp.rotation.z = rot.roll
            temp.lane_state = abs(p.lane_id)
            lpmsg.append(temp)
            # draw left and right lane markers
            width = p.lane_width
            next_p = None
            if i != len(lp) - 1:
                next_p = lp[i+1]
                vec = next_p.transform.location - loc
                if vec.y == 0:
                    mark1 = carla.Location(loc.x+width/2, loc.y,loc.z)
                    mark2 = carla.Location(loc.x-width/2, loc.y,loc.z)
                    percep_mod.world.debug.draw_point(mark1,life_time=2)
                    percep_mod.world.debug.draw_point(mark2,life_time=2)
                else:    
                    mark1, mark2 = get_markers(loc, vec, width)
                    percep_mod.world.debug.draw_point(mark1,life_time=2)
                    percep_mod.world.debug.draw_point(mark2,life_time=2)
            percep_mod.world.debug.draw_point(trans.location,life_time=2)
        obs_pub.publish(obsmsg)
        lane_pub.publish(lpmsg)
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
    try:
        publisher(pm, role_name)
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down raceinfo publisher")
