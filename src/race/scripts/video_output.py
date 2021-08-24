
import sys
import copy
import time
import rospy
import rospkg

import cv2
import math
import numpy as np

from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
from sensor_msgs.msg import NavSatFix
from cv_bridge import CvBridge, CvBridgeError


class VideoOutput:
    def __init__(self, role_name='ego_vehicle'):
        self.bridge = CvBridge()
        self.videoPub = rospy.Publisher("/carla/%s/video_output"%role_name, Image, queue_size=None)
        self.videoSub = rospy.Subscriber("/carla/%s/rgb_view/image"%role_name, Image, self.imageCallback)
        self.scoreSub = rospy.Subscriber("/carla/%s/score"%role_name, Float32, self.scoreCallback)
        self.reachedSub = rospy.Subscriber("/carla/%s/reached"%role_name, String, self.reachedCallback)
        self.collisionSub = rospy.Subscriber("/carla/%s/collision_detail"%role_name, String, self.collisionCallback)

        self.role_name = role_name
        # Default value 
        self.score = 0.0
        self.reachedInfo = ""
        self.collisionInfo = ""

        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.orgScore = (30, 30)
        self.orgReached = (30, 60)
        self.orgCollision = (30, 90)

        self.fontScale = 0.6
        self.color = (255, 0, 0) 
        self.thickness = 1
    
    def imageCallback(self, data):
        try:
		    # Convert ROS image to OpenCV image
            rawImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        img = np.copy(rawImage)

        scoreStr = "Score: {:.2f}".format(self.score)
        reachedStr = "Reached: {}".format(self.reachedInfo)
        collisionStr = "Collide with {}".format(self.collisionInfo)

        outImg = cv2.putText(img, scoreStr, self.orgScore, self.font, self.fontScale, self.color, self.thickness, cv2.LINE_AA) 
        outImg = cv2.putText(outImg, reachedStr, self.orgReached, self.font, self.fontScale, self.color, self.thickness, cv2.LINE_AA)
        outImg = cv2.putText(outImg, collisionStr, self.orgCollision, self.font, self.fontScale, self.color, self.thickness, cv2.LINE_AA)

        try: 
            self.videoPub.publish(self.bridge.cv2_to_imgmsg(outImg, "bgr8"))
        except CvBridgeError as e:
            rospy.logwarn("Video published failed with error: %s"%e)

    
    def scoreCallback(self, data):
        self.score = data.data

    def reachedCallback(self, data):
        self.reachedInfo = data.data

    def collisionCallback(self, data):
        self.collisionInfo = data.data
        

def run(role_name):
    VO = VideoOutput(role_name=role_name)
    rospy.Rate(20) 
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node("VideoOutput_Node")
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    rospy.loginfo("Start video output for %s!"%role_name)
    try:
        run(role_name)
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down video output node")
    