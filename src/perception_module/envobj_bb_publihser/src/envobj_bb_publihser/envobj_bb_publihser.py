#!/usr/bin/env python3
from typing import Counter
import carla
import rospy
import sys
from popgri_msgs.msg import BBInfo
from popgri_msgs.msg import BBList
from popgri_msgs.msg import BBSingleInfo
class PerceptionModule_BB():
    def __init__(self, carla_world, radius=10):
        self.sensing_radius = radius # default ?????
        self.world = carla_world
        self.vehicle = None
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
    #determine if the given bounding box is within sensing radius of the vehicle
    def bb_within_range(self, bb, self_location):
        radius = self.sensing_radius
        bb_loc = bb.location
        # project the center of the box to ground
        labelled_point = self.world.ground_projection(bb_loc, abs(bb.extent.z)/2)
        projected_loc = labelled_point.location
        if projected_loc.distance(self_location) <= radius:
            return True
        return False
    # TODO: return bounding box of environment objects within range
    # get city objects in terms of info on their bounding box
    # return type:carla.BoundingBox
    def get_bb_within_range(self, obj_type):
        # TODO: need to check validity of object type
        all_env_obj = self.world.get_environment_objects(obj_type)
        vehicle = self.vehicle
        self_loc = vehicle.get_location()
        filtered_obstacles = []
        for obj in all_env_obj:
            box = obj.bounding_box
            # TODO: check local VS global 
            if self.bb_within_range(box, self_loc):
                filtered_obstacles.append(box)
        return filtered_obstacles

# publish obstacles and lane waypoints information
def publisher(percep_mod, label_list):
    # main function
    pub = rospy.Publisher('environment_obj_bb', BBList)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        for label in label_list:
            bbs = percep_mod.get_bb_within_range(label)
            bbs_msgs = BBList()
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
    host = rospy.get_param("/carla/host", "127.0.0.1")
    port = rospy.get_param("/carla/host", 2000)
    timeout = rospy.get_param("/carla/timeout", 10)
    client = carla.Client(host=host, port=port)
    client.set_timeout(timeout)
    world = client.get_world()
    pm = PerceptionModule_BB(world)
    default_list = [carla.CityObjectLabel.Vehicles, carla.CityObjectLabel.Buildings, carla.CityObjectLabel.Fences, carla.CityObjectLabel.Pedestrians, carla.CityObjectLabel.Sidewalks, carla.CityObjectLabel.Walls, carla.CityObjectLabel.Vegetation, carla.CityObjectLabel.GuardRail]
    publisher(pm, default_list)
