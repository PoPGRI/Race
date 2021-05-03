import rospy 
import numpy as np
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int16
import time
from ackermann_msgs.msg import AckermannDrive
import os

class AckermannNode:

    def __init__(self, role_name='ego_vehicle'):
        self.subControl = rospy.Subscriber('/carla/%s/ackermann_control'%role_name, AckermannDrive, self.controlCallback)
        self.pubControl = rospy.Publisher('/carla/%s/ackermann_cmd'%role_name, AckermannDrive, queue_size=1)
        self.role_name = role_name

    def controlCallback(self, data):
        newAckermannCmd = data
        newAckermannCmd.steering_angle = -data.steering_angle
        newAckermannCmd.steering_angle_velocity = -data.steering_angle_velocity

        self.pubControl.publish(newAckermannCmd)

if __name__ == "__main__":
    rospy.init_node("Ackermann_Node")
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    # rospy.loginfo("Start evaluating the performance of %s!"%role_name)
    an = AckermannNode(role_name=role_name)
    rospy.spin()

