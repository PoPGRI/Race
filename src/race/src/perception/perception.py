import rospy
import numpy as np
from popgri_msgs.msg import ObstacleList, ObstacleInfo
from derived_object_msgs.msg import ObjectArray
import copy

def quaternion_to_euler(quat):
    x, y, z, w = quat
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = np.arcsin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(t3, t4)
    return [roll, pitch, yaw]

class VehiclePerception:
    def __init__(self, test=False):
        self.locationSub = rospy.Subscriber("/carla/objects", ObjectArray, self.locationCallback)
        self.obstacleSub = rospy.Subscriber("/obstacles", ObstacleList, self.obstacleCallback)
        self.position = None
        self.velocity = None 
        self.quat = None
        self.obstacleList = None

        self.test=test
        
    def locationCallback(self, data):
        
        self.position = (data.objects[0].pose.position.x, data.objects[0].pose.position.y)
        self.quat = (data.objects[0].pose.orientation.x, data.objects[0].pose.orientation.y, data.objects[0].pose.orientation.z, data.objects[0].pose.orientation.w)
        self.velocity = (data.objects[0].twist.linear.x, data.objects[0].twist.linear.y)

    def obstacleCallback(self, data):
        self.obstacleList = data.obstacles
        print(self.test)
        if self.test:
            curr_x = self.position[0]
            curr_y = self.position[1]
            # print(self.obstacleList)
            # print(curr_x, curr_y)
            for obs in self.obstacleList:
                psi = np.tan((curr_y - obs.location.y)/(curr_x - obs.location.x ))
                yaw = quaternion_to_euler(self.quat)[2]
                print(psi)
                print(yaw)
                if abs(psi-yaw) < 0.463:
                    print("Detected: ", obs)

if __name__ == "__main__":
    rospy.init_node("perception_node")
    VehiclePerception(test=True)

    rospy.spin()