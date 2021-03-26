#!/usr/bin/env python3
import sys
import copy
import time
import rospy
import rospkg

import cv2
import math
import numpy as np
import os

import ffmpeg as ff
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
from sensor_msgs.msg import NavSatFix
from cv_bridge import CvBridge, CvBridgeError
# reference:
# https://gist.github.com/rethink-imcmahon/77a1a4d5506258f3dc1f
class VideoGeneration:
    def __init__(self, role_name, path):
         self.bridge = CvBridge()
         self.role_name = role_name
         self.counter = 0
         self.path = path
         self.image_sub = rospy.Subscriber("/carla/hero0/video_output", Image, self.image_callback)
    def image_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            img_name = "frame{:04d}.jpg".format(self.counter)
            # print(img_name)
            cv2.imwrite(os.path.join(self.path, img_name), img)
            self.counter += 1
    def generate_video_from_images(self):
        while not rospy.is_shutdown():
            pass
        (
            ff
            .input(os.path.join(self.path, "*.jpg"), pattern_type='glob', framerate=20)
            .output("./video_generation/output_{}_{}.mp4".format(self.role_name, time.asctime().replace(' ', '_').replace(':', '_')))
            .run()
        )

        # os.system("./video_generation/clean.sh")
        # p=os.path.join("/home/carla/graic-workspace/src/race/scripts/video_generation/", "output.mp4")
        # os.system("ffmpeg -framerate 25 -i frame%04d.jpg -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p " + p)
    
if __name__ == "__main__":
    rospy.init_node("VideoGeneration_Node")
    role_name = rospy.get_param("~role_name", "hero0")

    
    # path = os.getcwd() + '/video_generation/images/'
    # print(path)
    os.chdir(os.path.dirname('/home/carla/'))
    # cwd = os.getcwd()
    path = os.getcwd() + '/video_generation/images/'
    vg = VideoGeneration(role_name, path)
    
    try:
        vg.generate_video_from_images()
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down video generation node")
