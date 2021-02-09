import rospy
import numpy as np
import argparse

from gazebo_msgs.msg import  ModelState
from controller.controller import VehicleController
from perception.perception import VehiclePerception
from decision.decision import VehicleDecision
import time
from util.waypoint_list import wayPoints
from util.util import euler_to_quaternion, quaternion_to_euler
import pickle



def run_model():
    rospy.init_node("baseline")
    rate = rospy.Rate(100)  # 100 Hz    

    perceptionModule = VehiclePerception()
    decisionModule = VehicleDecision('./waypoints')
    controlModule = VehicleController()

    def shut_down():
        controlModule.stop()
    rospy.on_shutdown(shut_down)

    while not rospy.is_shutdown():
        # res = sensors.lidarReading()
        # print(res)
        rate.sleep()  # Wait a while before trying to get a new state

        obstacleList = perceptionModule.obstacleList

        # Get the current position and orientation of the vehicle
        currState =  (perceptionModule.position, perceptionModule.rotation, perceptionModule.velocity)
        if not currState:
            continue
        print("Currently at: ", currState)
        # perceptionResult = perceptionModule.lidarReading()

        refState = decisionModule.get_ref_state(currState, obstacleList)
        if not refState:
            controlModule.stop()
            exit(0)
        print("target: ", refState)

        controlModule.execute(currState, refState)

if __name__ == "__main__":
    try:
        run_model()
    except rospy.exceptions.ROSInterruptException:
        print("stop")
    