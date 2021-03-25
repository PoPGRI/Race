import rospy 
import numpy as np
import time
from carla_msgs.msg import CarlaEgoVehicleControl
import os

class ControlNode:

    def __init__(self, role_name='ego_vehicle'):
        self.subControl = rospy.Subscriber('/carla/%s/vehicle_control'%role_name, CarlaEgoVehicleControl, self.controlCallback)
        self.pubControl = rospy.Publisher('/carla/%s/vehicle_control_cmd'%role_name, CarlaEgoVehicleControl, queue_size=None)
        self.role_name = role_name

    def controlCallback(self, data):
        newControlCmd = data
        newControlCmd.steer = data.steer
        self.pubControl.publish(newControlCmd)

if __name__ == "__main__":
    rospy.init_node("Control_Node")
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    cn = ControlNode(role_name=role_name)
    rospy.spin()

