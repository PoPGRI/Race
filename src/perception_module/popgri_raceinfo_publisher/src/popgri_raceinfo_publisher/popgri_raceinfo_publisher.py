#!/usr/bin/env python3
import carla
import rospy
import sys
from popgri_msgs.msg import LaneInfo
from popgri_msgs.msg import LaneList
from popgri_msgs.msg import ObstacleInfo
from popgri_msgs.msg import ObstacleList
class PerceptionModule():
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
    # return all the obstacles within the sensing radius of the vehicle
    def get_all_obstacles_within_range(self):
        # get every actor on stage
        if self.vehicle == None:
            rospy.loginfo("No ego vehicle.")
            return
        vehicle = self.vehicle
        vehicle_loc = vehicle.location
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
                if 'vehicle' in actor.type_id:
                    filtered_obstacles.append(actor)
                elif 'walker' in actor.type_id:
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
    def get_lane_way_point(self, distance=1.0):
        if self.vehicle == None:
            rospy.loginfo("No ego vehicle.")
            return
        vehicle = self.vehicle
        carla_map = self.world.get_map()
        vehicle_location = vehicle.get_location()
        # get a nearest waypoint
        cur_waypoint = carla_map.get_waypvoint(vehicle_location)
        # return list of waypoints from cur_waypoint to the end of the lane
        return cur_waypoint.next_until_lane_end(distance)

# publish obstacles and lane waypoints information
def publisher(percep_mod):
    # main function
    obs_pub = rospy.Publisher('obstacles', ObstacleList)
    lane_pub = rospy.Publisher('lane_waypoints', LaneList)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        obs = percep_mod.get_all_obstacles_within_range()
        lp = percep_mod.get_lane_way_point()
        obsmsg = []
        lpmsg = []
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
            obsmsg.append(temp)
        for p in lp:
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
            lpmsg.append(temp)
        obs_pub.publish(obsmsg)
        lane_pub.publish(lpmsg)
        rate.sleep()


if __name__ == "__main__":
    # reference: https://github.com/SIlvaMFPedro/ros_bridge/blob/master/carla_waypoint_publisher/src/carla_waypoint_publisher/carla_waypoint_publisher.py
    rospy.init_node('popgri_raceinfo_publisher', anonymous=True)
    # host = rospy.get_param("/carla/host", "127.0.0.1")
    # port = rospy.get_param("/carla/host", 2000)
    # timeout = rospy.get_param("/carla/timeout", 10)
    client = carla.Client('localhost', 2000)
    # client.set_timeout(timeout)
    world = client.get_world()
    pm = PerceptionModule(world)
    publisher(pm)
