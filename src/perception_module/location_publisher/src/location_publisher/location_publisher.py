#!/usr/bin/env python3
import carla
import rospy
import sys

from popgri_msgs.msg import LocationInfo

class LocationModule:
    def __init__(self, carla_world, role_name='ego_vehicle'):
        self.world = carla_world
        self.role_name = role_name
        self.vehicle = None
        self.find_ego_vehicle()

    def find_ego_vehicle(self):
        for actor in self.world.get_actors():
            if actor.attributes.get('role_name') == self.role_name:
                self.vehicle = actor
                break

    def getName(self):
        return self.vehicle.type_id 

    def getId(self):
        return self.vehicle.id

    def getLocation(self):
        return self.vehicle.get_location() 
    
    def getRotation(self):
        return self.vehicle.get_transform().rotation

    def getVelocity(self):
        return self.vehicle.get_velocity()

def publisher(location_module, role_name):
    # main function
    pub = rospy.Publisher('/carla/%s/location'%role_name, LocationInfo, queue_size=1)
    rate = rospy.Rate(20)
    while not location_module.vehicle:
        location_module.find_ego_vehicle()
        continue
    while not rospy.is_shutdown():
        info = LocationInfo()
        info.actor_name = location_module.getName()
        info.actor_id = location_module.getId()
        location = location_module.getLocation()
        info.location.x = location.x 
        info.location.y = location.y 
        info.location.z = location.z 
        rotation = location_module.getRotation()
        info.rotation.x = rotation.roll
        info.rotation.y = rotation.pitch
        info.rotation.z = rotation.yaw
        velocity = location_module.getVelocity()
        info.velocity.x = velocity.x
        info.velocity.y = velocity.y 
        info.velocity.z = velocity.z
        pub.publish(info)
    rate.sleep()


if __name__ == "__main__":
    # reference: https://github.com/SIlvaMFPedro/ros_bridge/blob/master/carla_waypoint_publisher/src/carla_waypoint_publisher/carla_waypoint_publisher.py
    rospy.init_node('popgri_raceinfo_publisher', anonymous=True)
    # host = rospy.get_param("/carla/host", "127.0.0.1")
    # port = rospy.get_param("/carla/host", 2000)
    # timeout = rospy.get_param("/carla/timeout", 10)
    # role_name = rospy.get_param('role_name', 'ego_vehicle')
    client = carla.Client('localhost', 2000)
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    # client.set_timeout(timeout)
    world = client.get_world()
    lm = LocationModule(world, role_name)    
    try:
        publisher(lm, role_name)
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down location publisher")