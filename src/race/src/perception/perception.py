import rospy
import numpy as np
from popgri_msgs.msg import ObstacleList, ObstacleInfo
from derived_object_msgs.msg import ObjectArray
from popgri_msgs.msg import LocationInfo
import copy

class VehiclePerception:
    def __init__(self, test=False):
        self.locationSub = rospy.Subscriber("/location", LocationInfo, self.locationCallback)
        self.obstacleSub = rospy.Subscriber("/obstacles", ObstacleList, self.obstacleCallback)
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
        # print(self.test)
        if self.test:
            curr_x = self.position[0]
            curr_y = self.position[1]
            for obs in self.obstacleList:
                dy = obs.location.y - curr_y
                dx = obs.location.x - curr_x
                yaw = self.rotation[2] 
                rx = np.cos(-yaw) * dx - np.sin(-yaw) * dy 
                ry = np.cos(-yaw) * dy + np.sin(-yaw) * dx
 
                print(rx)
                print(ry)
                psi = np.arctan(ry/rx)
                print(psi)
                if rx > 0 and abs(psi) < 0.463:
                    print("Detected: ", obs)

if __name__ == "__main__":
    rospy.init_node("perception_node")
    VehiclePerception(test=True)

    rospy.spin()