import rospy
import rospkg
import numpy as np
import argparse
import time
from graic_msgs.msg import ObstacleList, ObstacleInfo
from graic_msgs.msg import LocationInfo
from ackermann_msgs.msg import AckermannDrive
from carla_msgs.msg import CarlaEgoVehicleControl
from graic_msgs.msg import LaneList
from graic_msgs.msg import LaneInfo


class VehicleController():

    def __init__(self, role_name='ego_vehicle'):
        # Publisher to publish the control input to the vehicle model
        self.role_name = role_name
        self.controlPub = rospy.Publisher("/carla/%s/ackermann_control"%role_name, AckermannDrive, queue_size = 1)

    def execute(self):
        # TODO Add your controller code
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = 0
        newAckermannCmd.steering_angle = 0
        self.controlPub.publish(newAckermannCmd)

class VehicleDecision():
    def __init__(self, role_name='ego_vehicle'):
        self.role_name = role_name

    # TODO Add your decision logic here

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

    print("Starter code is running")

    while not rospy.is_shutdown():
        rate.sleep()  # Wait a while before trying to get a new state
        obstacleList = perceptionModule.obstacleList

        # Get the current position and orientation of the vehicle
        currState =  (perceptionModule.position, perceptionModule.rotation, perceptionModule.velocity)
        if not currState or not currState[0]:
            continue

        # Execute 
        controlModule.execute()

if __name__ == "__main__":
    roskpack = rospkg.RosPack() 
    config_path = roskpack.get_path('config_node')
    race_config = open(config_path+'/'+'race_config', 'rb')
    vehicle_typeid = race_config.readline().decode('ascii').strip()
    sensing_radius = race_config.readline().decode('ascii').strip()
    role_name = 'ego_vehicle'

    rospy.init_node("baseline")
    try:
        run_model(role_name)
    except rospy.exceptions.ROSInterruptException:
        print("stop")
    