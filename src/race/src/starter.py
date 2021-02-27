import rospy
import numpy as np
import argparse
import time
from popgri_msgs.msg import ObstacleList, ObstacleInfo
from popgri_msgs.msg import LocationInfo
from ackermann_msgs.msg import AckermannDrive
from carla_msgs.msg import CarlaEgoVehicleControl
from popgri_msgs.msg import LaneList
from popgri_msgs.msg import LaneInfo


class VehicleController():

    def __init__(self, role_name='ego_vehicle'):
        # Publisher to publish the control input to the vehicle model
        self.controlPub = rospy.Publisher("/carla/%s/ackermann_control"%role_name, AckermannDrive, queue_size = 1)

    def execute(self):
        # TODO Add your controller code
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = 0
        newAckermannCmd.steering_angle = 0
        self.controlPub.publish(newAckermannCmd)

class VehiclePerception:
    def __init__(self, role_name='ego_vehicle'):
        self.locationSub = rospy.Subscriber("/carla/%s/location"%role_name, LocationInfo, self.locationCallback)
        self.obstacleSub = rospy.Subscriber("/carla/%s/obstacles"%role_name, ObstacleList, self.obstacleCallback)
        self.position = None
        self.velocity = None 
        self.rotation = None
        self.obstacleList = None

        
    def locationCallback(self, data):
        self.position = (data.location.x, data.location.y)
        self.rotation = (np.radians(data.rotation.x), np.radians(data.rotation.y), np.radians(data.rotation.z))
        self.velocity = (data.velocity.x, data.velocity.y)

    def obstacleCallback(self, data):
        self.obstacleList = data.obstacles


def run_model(role_name):
    
    rate = rospy.Rate(100)  # 100 Hz    

    perceptionModule = VehiclePerception(role_name=role_name)
    decisionModule = VehicleDecision(role_name)
    controlModule = VehicleController(role_name=role_name)

    def shut_down():
        controlModule.stop()
    rospy.on_shutdown(shut_down)

    while not rospy.is_shutdown():
        rate.sleep()  # Wait a while before trying to get a new state
        obstacleList = perceptionModule.obstacleList

        # Get the current position and orientation of the vehicle
        currState =  (perceptionModule.position, perceptionModule.rotation, perceptionModule.velocity)
        if not currState or not currState[0]:
            continue

        # TODO Add your decision logic here

        # Execute 
        controlModule.execute()

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
    