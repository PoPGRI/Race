#!/usr/bin/env python3
from typing import Counter
import carla
import rospy
# import sys
import numpy as np
from popgri_msgs.msg import BBInfo
from popgri_msgs.msg import BBList
from popgri_msgs.msg import BBSingleInfo
class PerceptionModule_BB():
    def __init__(self, carla_world, radius=10):
        self.sensing_radius = radius # default ?????
        self.world = carla_world
        self.vehicle = None
        self.find_ego_vehicle()
    # find ego vehicle
    # reference: 
    def find_ego_vehicle(self):
        for actor in self.world.get_actors():
            if actor.attributes.get('role_name') == 'ego_vehicle':
                self.vehicle = actor
                break

    def set_radius(self, new_radius):
        self.sensing_radius = new_radius
    def get_radius(self):
        return self.sensing_radius
    # transform from local coordinate to world coordinate
    def obj_to_world(self, cords, obj):
        trans = obj.transform
        trans.transform(cords)
        return cords
    #determine if the given bounding box is within sensing radius of the vehicleh 
    # input:obj: the object we want to detect
    #       bb: the objtec's bounding box
    #       self_locsation: the location of the ego_vehicle
    def bb_within_range(self, obj, bb, self_location):
        radius = self.sensing_radius
        bb_loc_local = bb.location
        bb_loc_global = self.obj_to_world(bb_loc_local, obj)
        # vehicle_loc = self.vehicle.get_location()
        
        # get the world coordinate of the box center
        # world_cord_box = self.obj_to_world(bb_loc, bb)
        
        # cast ray returns all points intersecting the ray between initial location and final location
        # for descirption of the method, see: https://carla.readthedocs.io/en/latest/python_api/#carlaworld
        labelled_points_in_way = self.world.cast_ray(self_location, bb_loc_global)
        # remove points outside of range
        for point in labelled_points_in_way:
            if point.location.distance(self_location) > radius:
                labelled_points_in_way.remove(point)
        for point in labelled_points_in_way:
            # if bounding box contains any point in the way, then 
            if bb.contains(point.location, obj.transform):
                return True
        return False
    # TODO: return bounding box of environment objects within range
    # get city objects in terms of info on their bounding box
    # return type:carla.BoundingBox
    def get_bb_within_range(self, obj_type):
        # TODO: need to check validity of object type
        if self.vehicle == None:
            self.find_ego_vehicle()
            # rospy.loginfo("No ego vehicle.")
            return
        all_env_obj = self.world.get_environment_objects(obj_type)
        vehicle = self.vehicle
        self_loc = vehicle.get_location()
        filtered_obstacles = []
        for obj in all_env_obj:
            box = obj.bounding_box
            if self.bb_within_range(obj, box, self_loc):
                filtered_obstacles.append(box)
        return filtered_obstacles

# publish obstacles and lane waypoints information
def publisher(percep_mod, label_list):
    # main function
    pub = rospy.Publisher('environment_obj_bb', BBList, queue_size=1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        for label in label_list:
            bbs = percep_mod.get_bb_within_range(label)
            bbs_msgs = BBList()
            if not bbs:
                continue
            for bb in bbs:
                info = BBInfo()
                #get local coordinates(global coordinates need parameter carla.Transform)
                local_vertices = bb.get_local_vertices()
                for loc in local_vertices:
                    vertex = BBSingleInfo()
                    vertex.vertex_location.x = loc.x
                    vertex.vertex_location.y = loc.y
                    vertex.vertex_location.z = loc.z
                    info.append(vertex)
                info.type = label
                bbs_msgs.append(info)
            pub.publish(bbs_msgs)
        rate.sleep()


if __name__ == "__main__":
    # reference: https://github.com/SIlvaMFPedro/ros_bridge/blob/master/carla_waypoint_publisher/src/carla_waypoint_publisher/carla_waypoint_publisher.py
    rospy.init_node('envobj_bb_publihser', anonymous=True)
    # host = rospy.get_param("/carla/host", "127.0.0.1")
    # port = rospy.get_param("/carla/host", 2000)
    # timeout = rospy.get_param("/carla/timeout", 10)
    client = carla.Client('localhost', 2000)
    # client.set_timeout(timeout)
    world = client.get_world()
    pm = PerceptionModule_BB(world)
    default_list = [carla.CityObjectLabel.Vehicles, carla.CityObjectLabel.Buildings, carla.CityObjectLabel.Fences, carla.CityObjectLabel.Pedestrians, carla.CityObjectLabel.Sidewalks, carla.CityObjectLabel.Walls, carla.CityObjectLabel.Vegetation, carla.CityObjectLabel.GuardRail]
    publisher(pm, default_list)
