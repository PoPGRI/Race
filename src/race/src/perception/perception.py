import rospy
import numpy as np
from popgri_msgs.msg import ObstacleList, ObstacleInfo
from derived_object_msgs.msg import ObjectArray
from popgri_msgs.msg import LocationInfo
import copy

class VehiclePerception:
    def __init__(self, role_name='ego_vehicle', test=False):
        self.locationSub = rospy.Subscriber("/carla/%s/location"%role_name, LocationInfo, self.locationCallback)
        self.obstacleSub = rospy.Subscriber("/carla/%s/obstacles"%role_name, ObstacleList, self.obstacleCallback)
        self.position = None
        self.velocity = None 
        self.rotation = None
        self.obstacleList = None

        self.test=test
        
    def locationCallback(self, data):
        self.position = (data.location.x, data.location.y)
        self.rotation = (np.radians(data.rotation.x), np.radians(data.rotation.y), np.radians(data.rotation.z))
        self.velocity = (data.velocity.x, data.velocity.y)

    def obstacleCallback(self, data):
        self.obstacleList = data.obstacles

if __name__ == "__main__":
    rospy.init_node("perception_node")
    VehiclePerception(role_name='hero0', test=True)

    rospy.spin()