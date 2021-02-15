import rospy
import numpy as np
import argparse

from gazebo_msgs.msg import  ModelState
from controller.controller import VehicleController
from perception.perception import VehiclePerception
from decision.decision import VehicleDecision
import time
from util.util import euler_to_quaternion, quaternion_to_euler
import pickle



def run_model(role_name):
    
    rate = rospy.Rate(100)  # 100 Hz    

    perceptionModule = VehiclePerception(role_name=role_name)
    decisionModule = VehicleDecision('./waypoints')
    controlModule = VehicleController(role_name=role_name)

    def shut_down():
        controlModule.stop()
    rospy.on_shutdown(shut_down)

    while not rospy.is_shutdown():
        rate.sleep()  # Wait a while before trying to get a new state
        obstacleList = perceptionModule.obstacleList

        # Get the current position and orientation of the vehicle
        currState =  (perceptionModule.position, perceptionModule.rotation, perceptionModule.velocity)
        if not currState:
            continue
        print("Currently at: ", currState)
        
        # Get the target state from decision module
        refState = decisionModule.get_ref_state(currState, obstacleList)
        if not refState:
            controlModule.stop()
            exit(0)
        print("target: ", refState)

        # Execute 
        controlModule.execute(currState, refState)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Running vechile")

    role_name_default = 'ego_vehicle'

    parser.add_argument('--name', type=str, help='Rolename of the vehicle', default=role_name_default)
    argv = parser.parse_args()
    role_name = argv.name
    rospy.init_node("baseline")
    role_name = 'hero0'
    try:
        run_model(role_name)
    except rospy.exceptions.ROSInterruptException:
        print("stop")
    