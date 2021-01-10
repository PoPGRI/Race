import sys
import os
import argparse

import numpy as np

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import GetModelState

def getModelState():
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        modelState = serviceResponse(model_name='gem')
    except rospy.ServiceException as exc:
        rospy.loginfo("Service did not process request: "+str(exc))
    return modelState

def setModelState(model_state):
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state(model_state)
    except rospy.ServiceException as e:
        rospy.loginfo("Service did not process request: "+str(e))

def euler_to_quaternion(r):
    (yaw, pitch, roll) = (r[0], r[1], r[2])
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

def set_position(x = 0,y = 0, name='gem'):
    
    rospy.init_node("set_pos")

    curr_state = getModelState()
    new_state = ModelState()
    new_state.model_name = name
    new_state.twist.linear.x = 0
    new_state.twist.linear.y = 0
    new_state.twist.linear.z = 0
    new_state.pose.position.x = x
    new_state.pose.position.y = y
    new_state.pose.position.z = 1
    q = euler_to_quaternion([0,0,0])
    new_state.pose.orientation.x = q[0]
    new_state.pose.orientation.y = q[1]
    new_state.pose.orientation.z = q[2]
    new_state.pose.orientation.w = q[3]
    new_state.twist.angular.x = 0
    new_state.twist.angular.y = 0
    new_state.twist.angular.z = 0
    setModelState(new_state)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description = 'Set the x, y position of the vehicle')

    x_default = -10
    y_default = -98

    parser.add_argument('--x', type = float, help = 'x position of the vehicle.', default = x_default)
    parser.add_argument('--y', type = float, help = 'y position of the vehicle.', default = y_default)

    argv = parser.parse_args()

    x = argv.x
    y = argv.y

    set_position(y = -98, x = 0, name='gem')
