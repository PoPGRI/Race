#!/usr/bin/env python

import sys
import copy
import time
import rospy
import rospkg

import cv2
import math
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import NavSatFix
from cv_bridge import CvBridge, CvBridgeError


PI = 3.1415926535

# 30Hz
SPIN_RATE = 15 

class ImageConverter:

    def __init__(self, SPIN_RATE):

        self.bridge = CvBridge()

        self.image_sensor_pub = rospy.Publisher("/gem/sensor_image", Image, queue_size=10)
        self.image_sub        = rospy.Subscriber("/gem/front_single_camera/front_single_camera/image_raw", Image, self.image_callback)
        self.gps_sub          = rospy.Subscriber("/gps/fix", NavSatFix, self.gps_callback)

        self.gps_msg          = NavSatFix()
        self.loop_rate        = rospy.Rate(SPIN_RATE)

        self.font             = cv2.FONT_HERSHEY_SIMPLEX 
        self.org_lat          = (30, 30) 
        self.org_lon          = (30, 60) 
        self.fontScale        = 0.6
        self.color            = (255, 0, 0) 
        self.thickness        = 1

        self.latitude         = 0.0
        self.longitude        = 0.0

        # Check if ROS is ready for operation
        while(rospy.is_shutdown()):
        	print("ROS is shutdown!")


    def image_callback(self, data):

        try:
		  # Convert ROS image to OpenCV image
            raw_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # In BGR format
        lane_img = np.copy(raw_image)

        lat = "Latitude: " + str(self.latitude)
        lon = "Longitude: " + str(self.longitude)

        # Using cv2.putText() method 
        out_img = cv2.putText(lane_img, lat, self.org_lat, self.font, self.fontScale, self.color, self.thickness, cv2.LINE_AA) 
        out_img = cv2.putText(out_img, lon, self.org_lon, self.font, self.fontScale, self.color, self.thickness, cv2.LINE_AA) 

        try:
		  # Convert OpenCV image to ROS image and publish
            self.image_sensor_pub.publish(self.bridge.cv2_to_imgmsg(out_img, "bgr8"))
        except CvBridgeError as e:
            print(e)


    def gps_callback(self, data):
    	self.latitude  = round(data.latitude, 6)
    	self.longitude = round(data.longitude, 6)





"""
Program run from here
"""
def main():

	# Initialize ROS node
	rospy.init_node('gem_sensor_image')

	# Check if ROS is ready for operation
	while(rospy.is_shutdown()):
		rospy.loginfo("ROS is shutdown!")

	# Initialize the rate to publish to ur3/command
	loop_rate = rospy.Rate(SPIN_RATE)

	ic_sensor = ImageConverter(SPIN_RATE)

	rospy.loginfo("Use Ctrl+C to exit program")

	rospy.spin()

if __name__ == '__main__':
	
	try:
		main()
    # When Ctrl+C is executed, it catches the exception
	except rospy.ROSInterruptException:
		pass