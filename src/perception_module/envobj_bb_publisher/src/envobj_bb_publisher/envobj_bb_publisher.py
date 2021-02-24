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
    def __init__(self, carla_world, role_name='ego_vehicle', radius=15):
        self.sensing_radius = radius 
        self.world = carla_world
        self.vehicle = None
        self.role_name = role_name
        self.find_ego_vehicle()
    # find ego vehicle
    # reference: 
    def find_ego_vehicle(self):
        for actor in self.world.get_actors():
            if actor.attributes.get('role_name') == self.role_name:
                self.vehicle = actor
                break

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
        # otherwise calculate the distance using center of the vehicle and box
        trans = carla.Transform(carla.Location(0,0,0), carla.Rotation(0,0,0))
        # obtain a vector pointing from self_loc to box center in world space
        vec_from_self_to_box = box_loc - self_loc
        vec_from_self_to_box.z = 0
        normalized_vec = vec_from_self_to_box/self.distance_between_points(vec_from_self_to_box, vec_from_self_to_box)
        # calculate the location which is 'radius' away from the vehicle along the vec
        tip_of_vec = normalized_vec*radius
        loc_of_vec = tip_of_vec + self_loc
        if box.contains(loc_of_vec, trans):
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
                filtered_obstacles.append((env_bb.bounding_box.get_local_vertices(), env_bb.name, env_bb.id, env_bb))
            elif self.boundingbox_within_range(env_bb.bounding_box, self_loc):
                filtered_obstacles.append((env_bb.bounding_box.get_local_vertices(), env_bb.name, env_bb.id, env_bb))
        return filtered_obstacles

# publish obstacles and lane waypoints information
def publisher(percep_mod, label_list, role_name):
    # main function
    pub = rospy.Publisher('/carla/%s/environment_obj_bb'%role_name, BBList, queue_size=1)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        for label in label_list:
            # get all vertices of all bounding boxes which are within the radius with label 'label'
            vertices_of_cur_label = percep_mod.get_bb_global_ver_within_range(label)
            bbs_msgs = BBList()
            if not vertices_of_cur_label:
                continue
            for vertices_of_one_box in vertices_of_cur_label:
                info = BBInfo()
                for loc in vertices_of_one_box[0]:
                    vertex = BBSingleInfo()
                    vertex.vertex_location.x = loc.x
                    vertex.vertex_location.y = loc.y
                    vertex.vertex_location.z = loc.z
                    info.vertices_locations.append(vertex)
                info.type = str(label)
                info.name = str(vertices_of_one_box[1])
                info.id = vertices_of_one_box[2] % 1013 # NOTE 1013 is a magic number, the purpose is to shorten the id from a very large number
                bbs_msgs.bounding_box_vertices.append(info)
                bb = vertices_of_one_box[3].bounding_box
                percep_mod.world.debug.draw_box(bb, bb.rotation, life_time=0)
            pub.publish(bbs_msgs)
        rate.sleep()


if __name__ == "__main__":
    # reference: https://github.com/SIlvaMFPedro/ros_bridge/blob/master/carla_waypoint_publisher/src/carla_waypoint_publisher/carla_waypoint_publisher.py
    rospy.init_node('envobj_bb_publisher', anonymous=True)
    # host = rospy.get_param("/carla/host", "127.0.0.1")
    # port = rospy.get_param("/carla/host", 2000)
    # timeout = rospy.get_param("/carla/timeout", 10)
    client = carla.Client('localhost', 2000)
    # client.set_timeout(timeout)
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    # client.set_timeout(timeout)
    world = client.get_world()
    pm = PerceptionModule_BB(world, role_name=role_name)
    default_list = [carla.CityObjectLabel.Buildings, carla.CityObjectLabel.Fences, carla.CityObjectLabel.Sidewalks, carla.CityObjectLabel.Walls, carla.CityObjectLabel.Vegetation]
    try:
        publisher(pm, default_list, role_name)
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down environment object publisher")