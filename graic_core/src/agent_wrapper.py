import rospy
import rospkg
import numpy as np
import argparse
import time
from graic_msgs.msg import ObstacleList, ObstacleInfo
from graic_msgs.msg import LocationInfo, WaypointInfo
from ackermann_msgs.msg import AckermannDrive
from carla_msgs.msg import CarlaEgoVehicleControl
from graic_msgs.msg import LaneList
from graic_msgs.msg import LaneInfo

class VehiclePerception:
    def __init__(self, role_name='ego_vehicle', test=False):
        rospy.Subscriber("/carla/%s/location"%role_name, LocationInfo, self.locationCallback)
        rospy.Subscriber("/carla/%s/obstacles"%role_name, ObstacleList, self.obstacleCallback)
        rospy.Subscriber("/carla/%s/lane_markers"%role_name, LaneInfo, self.lanemarkerCallback)
        rospy.Subscriber("/carla/%s/waypoints"%role_name, WaypointInfo, self.waypointCallback)

        self.position = None
        self.rotation = None
        self.velocity = None
        self.obstacleList = None
        self.lane_marker = None
        self.waypoint = None

        self.test = test
        
    def locationCallback(self, data):
        self.position = (data.location.x, data.location.y)
        self.rotation = (np.radians(data.rotation.x), np.radians(data.rotation.y), np.radians(data.rotation.z))
        self.velocity = (data.velocity.x, data.velocity.y)

    def obstacleCallback(self, data):
        self.obstacleList = data.obstacles

    def lanemarkerCallback(self, data):
        self.lane_marker = data

    def waypointCallback(self, data):
        self.waypoint = data

    def ready(self):
        return self.position is not None and self.rotation is not None and self.velocity is not None and self.obstacleList is not None and self.waypoint is not None and self.lane_marker is not None

    def clear(self):
        self.position = None
        self.rotation = None
        self.velocity = None
        self.obstacleList = None
        self.lane_marker = None
        self.waypoint = None

def publish_control(pub_control, control):
    pub_control.publish_control(control)

def run_model(role_name, controller):
    perceptionModule = VehiclePerception(role_name=role_name)
    rate = rospy.Rate(100)

    controlPub = rospy.Publisher("/carla/%s/ackermann_control"%role_name, AckermannDrive, queue_size = 1)

    def shut_down():
        control = controller.stop()
        publish_control(controlPub, control)

    rospy.on_shutdown(shut_down)

    newAckermannCmd = AckermannDrive()
    newAckermannCmd.acceleration = 0
    newAckermannCmd.speed = 0
    newAckermannCmd.steering_angle = 0
    publish_control(controlPub, newAckermannCmd)

    while not rospy.is_shutdown():
        if perceptionModule.ready():
            # Get the current position and orientation of the vehicle
            currState =  (perceptionModule.position, perceptionModule.rotation, perceptionModule.velocity)
            control = controller.execute(currState, self.obstacleList)
            perceptionModule.clear()
            publish_control(controlPub, control)
        rate.sleep()  # Wait a while before trying to get a new state

if __name__ == "__main__":
    roskpack = rospkg.RosPack() 
    config_path = roskpack.get_path('graic_config')
    race_config = open(config_path+'/'+'race_config', 'rb')
    vehicle_typeid = race_config.readline().decode('ascii').strip()
    sensing_radius = race_config.readline().decode('ascii').strip()
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    class_file = rospy.get_param("~class_file", "")
    import importlib.util
    spec = importlib.util.spec_from_file_location("Controller", class_file)
    user_file = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(user_file)
    controller = user_file.Controller()
    rospy.init_node("graic_agent_wrapper", anonymous=True)
    try:
        run_model(role_name, controller)
    except rospy.exceptions.ROSInterruptException:
        print("stop")
    