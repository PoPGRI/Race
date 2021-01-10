#!/usr/bin/env python

import math
import numpy as np
import threading

import tf
import rospy

from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float64
from controller_manager_msgs.srv import ListControllers


PI = 3.141592653589739


def get_steer_angle(phi):
    if phi >= 0.0:
        return (PI / 2) - phi
    return (-PI / 2) - phi


class GEMController(object):

    def __init__(self):

        rospy.init_node("gem_ackermann_controller")

        self.right_rear_link = None

        (left_steer_link, left_steer_controller, left_front_wheel_controller, self.left_front_inv_circ) = \
            self.get_front_wheel_params("~left_front_wheel/")

        (right_steer_link, right_steer_controller, right_front_wheel_controller, self.right_front_inv_circ) = \
            self.get_front_wheel_params("~right_front_wheel/")

        (left_rear_link, left_rear_wheel_controller, self.left_rear_inv_circ) =  \
            self.get_rear_wheel_params("~left_rear_wheel/")

        (right_rear_link, right_rear_wheel_controller, self.right_rear_inv_circ) =  \
            self.get_rear_wheel_params("~right_rear_wheel/")

        self.right_rear_link = right_rear_link

        list_controllers = rospy.ServiceProxy("controller_manager/list_controllers", ListControllers)

        list_controllers.wait_for_service()

        self.cmd_timeout = float(rospy.get_param("~cmd_timeout", 0.5))

        self.sleep = rospy.Rate(float(rospy.get_param("~publishing_frequency", 30.0)))

        self.last_cmd_time = rospy.get_time()

        tfl = tf.TransformListener() 

        ls = self.get_link_position(tfl, left_steer_link) 

        rs = self.get_link_position(tfl, right_steer_link) 

        lrw= self.get_link_position(tfl, left_rear_link)

        rrw = np.array([0.0] * 3)

        self.steer_joint_dist_div_2 = np.linalg.norm(ls-rs)/2.0

        self.wheelbase = np.linalg.norm((ls+rs)/2.0 - (lrw+rrw)/2.0)

        self.wheelbase_inv = 1 / (self.wheelbase*1.0)

        self.wheelbase_sqr = self.wheelbase**2

        self.ackermann_cmd_lock = threading.Lock()

        self.steer_ang             = 0.0   
        self.steer_ang_vel         = 0.0   
        self.speed                 = 0.0
        self.accel                 = 0.0   
        self.last_steer_ang        = 0.0   
        self.theta_left            = 0.0   
        self.theta_left_old        = 0.0
        self.theta_right           = 0.0
        self.theta_right_old       = 0.0   
        self.last_speed            = 0.0
        self.last_accel_limit      = 0.0   
        self.left_front_ang_vel    = 0.0
        self.right_front_ang_vel   = 0.0
        self.left_rear_ang_vel     = 0.0
        self.right_rear_ang_vel    = 0.0

        self.left_steer_pub        = rospy.Publisher(left_steer_controller + "/command", Float64, queue_size=1)
        self.right_steer_pub       = rospy.Publisher(right_steer_controller + "/command", Float64, queue_size=1)
        self.left_front_wheel_pub  = rospy.Publisher(left_front_wheel_controller + "/command", Float64, queue_size=1)
        self.right_front_wheel_pub = rospy.Publisher(right_front_wheel_controller + "/command", Float64, queue_size=1)
        self.left_rear_wheel_pub   = rospy.Publisher(left_rear_wheel_controller + "/command", Float64, queue_size=1)
        self.right_rear_wheel_pub  = rospy.Publisher(right_rear_wheel_controller + "/command", Float64, queue_size=1)
        self.gem_ackermann_sub     = rospy.Subscriber("ackermann_cmd", AckermannDrive, self.ackermann_callback, queue_size=1)

    def spin(self):

        last_time = rospy.get_time()

        while not rospy.is_shutdown():

            t = rospy.get_time()
            delta_t = t - last_time
            last_time = t

            if (self.cmd_timeout > 0.0 and (t - self.last_cmd_time > self.cmd_timeout)):

                steer_ang_changed, center_y = self.control_steering(self.last_steer_ang, 0.0, 0.001)

                self.control_wheels(0.0, 0.0, 0.0, steer_ang_changed, center_y)

            elif delta_t > 0.0:

                with self.ackermann_cmd_lock:
                    steer_ang     = self.steer_ang
                    steer_ang_vel = self.steer_ang_vel
                    speed         = self.speed
                    accel         = self.accel

                steer_ang_changed, center_y = self.control_steering(steer_ang, steer_ang_vel, delta_t)

                self.control_wheels(speed, accel, delta_t, steer_ang_changed, center_y)

            # # change is bigger than 0.1 degree
            # if(np.abs(self.theta_left - self.theta_left_old) >= 0.0015):
            #     self.theta_left_old = self.theta_left
            #     self.left_steer_pub.publish(self.theta_left)

            # if(np.abs(self.theta_right - self.theta_right_old) >= 0.0015):
            #     self.theta_right_old = self.theta_right
            #     self.left_steer_pub.publish(self.theta_right)

            self.left_steer_pub.publish(self.theta_left)
            self.right_steer_pub.publish(self.theta_right)

            if self.left_front_wheel_pub:
                self.left_front_wheel_pub.publish(self.left_front_ang_vel)

            if self.right_front_wheel_pub:
                self.right_front_wheel_pub.publish(self.right_front_ang_vel)

            if self.left_rear_wheel_pub:
                self.left_rear_wheel_pub.publish(self.left_rear_ang_vel)

            if self.right_rear_wheel_pub:
                self.right_rear_wheel_pub.publish(self.right_rear_ang_vel)

            self.sleep.sleep()

    def ackermann_callback(self, ackermann_cmd):
        self.last_cmd_time = rospy.get_time()
        with self.ackermann_cmd_lock:
            self.steer_ang = ackermann_cmd.steering_angle
            self.steer_ang_vel = ackermann_cmd.steering_angle_velocity
            self.speed = ackermann_cmd.speed
            self.accel = ackermann_cmd.acceleration

    def get_front_wheel_params(self, prefix):
        steer_link = rospy.get_param(prefix + "steering_link_name")
        steer_controller = rospy.get_param(prefix + "steering_controller_name")
        wheel_controller = rospy.get_param(prefix + "axle_controller_name")
        diameter = float(rospy.get_param(prefix + "diameter"))
        return steer_link, steer_controller, wheel_controller, 1 / (PI * diameter)

    def get_rear_wheel_params(self, prefix):
        link = rospy.get_param(prefix + "link_name")
        wheel_controller = rospy.get_param(prefix + "axle_controller_name")
        diameter = float(rospy.get_param(prefix + "diameter"))
        return link, wheel_controller, 1 / (PI * diameter)

    def get_link_position(self, tfl, link):
        while True:
            try:
                trans, not_used = tfl.lookupTransform(self.right_rear_link, link, rospy.Time(0))
                return np.array(trans)
            except:
                pass

    def control_steering(self, steer_ang, steer_ang_vel_limit, delta_t):

        if steer_ang_vel_limit > 0.0:
            ang_vel = (steer_ang - self.last_steer_ang) / delta_t
            ang_vel = max(-steer_ang_vel_limit, min(ang_vel, steer_ang_vel_limit))
            theta = self.last_steer_ang + ang_vel * delta_t
        else:
            theta = steer_ang

        center_y = self.wheelbase * math.tan((PI/2)-theta)

        steer_ang_changed = theta != self.last_steer_ang

        if steer_ang_changed:
            self.last_steer_ang = theta
            self.theta_left = get_steer_angle(math.atan(self.wheelbase_inv * (center_y - self.steer_joint_dist_div_2)))
            self.theta_right = get_steer_angle(math.atan(self.wheelbase_inv * (center_y + self.steer_joint_dist_div_2)))

        return steer_ang_changed, center_y

    def control_wheels(self, speed, accel_limit, delta_t, steer_ang_changed, center_y):

        if accel_limit > 0.0:
            self.last_accel_limit = accel_limit
            accel = (speed - self.last_speed) / delta_t
            accel = max(-accel_limit, min(accel, accel_limit))
            veh_speed = self.last_speed + accel * delta_t
        else:
            self.last_accel_limit = accel_limit
            veh_speed = speed

        if veh_speed != self.last_speed or steer_ang_changed:
            self.last_speed = veh_speed
            left_dist = center_y - self.steer_joint_dist_div_2
            right_dist = center_y + self.steer_joint_dist_div_2
            gain = (2 * PI) * veh_speed / abs(center_y) 
            r = math.sqrt(left_dist ** 2 + self.wheelbase_sqr)
            self.left_front_ang_vel = gain * r * self.left_front_inv_circ
            r = math.sqrt(right_dist ** 2 + self.wheelbase_sqr)
            self.right_front_ang_vel = gain * r * self.right_front_inv_circ
            gain = (2 * PI) * veh_speed / center_y
            self.left_rear_ang_vel = gain * left_dist * self.left_rear_inv_circ
            self.right_rear_ang_vel = gain * right_dist * self.right_rear_inv_circ

if __name__ == "__main__":
    controller = GEMController()
    controller.spin()





