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


class VehicleDecision():
    def __init__(self):
        self.vehicle_state = 'straight'
        self.lane_state = 0
        self.counter = 0

        self.lane_marker = None
        self.target_x = None
        self.target_y = None
        self.change_lane = False
        self.change_lane_wp_idx = 0
        self.detect_dist = 30
        self.speed = 20

        self.reachEnd = False

    def get_ref_state(self, currState, obstacleList, lane_marker, waypoint):
        """
            Get the reference state for the vehicle according to the current state and result from perception module
            Inputs:
                currState: [Loaction, Rotation, Velocity] the current state of vehicle
                obstacleList: List of obstacles
            Outputs: reference state position and velocity of the vehicle
        """
        # self.reachEnd = waypoint.reachedFinal

        self.lane_marker = lane_marker.lane_markers_center.location[-1]
        self.lane_state = lane_marker.lane_state
        if not self.target_x or not self.target_y:
            self.target_x = self.lane_marker.x
            self.target_y = self.lane_marker.y
        if self.reachEnd:
            return None
        # print("Reach end: ", self.reachEnd)

        curr_x = currState[0][0]
        curr_y = currState[0][1]

        # Check whether any obstacles are in the front of the vehicle
        obs_front = False
        obs_left = False
        obs_right = False
        front_dist = 20
        if obstacleList:
            for obs in obstacleList:
                for vertex in obs.vertices_locations:
                    dy = vertex.vertex_location.y - curr_y
                    dx = vertex.vertex_location.x - curr_x
                    yaw = currState[1][2]
                    rx = np.cos(-yaw) * dx - np.sin(-yaw) * dy
                    ry = np.cos(-yaw) * dy + np.sin(-yaw) * dx

                    psi = np.arctan(ry / rx)
                    if rx > 0:
                        front_dist = np.sqrt(dy * dy + dx * dx)
                        # print("detected object is at {} away and {} radians".format(front_dist, psi))
                        if psi < 0.2 and psi > -0.2:
                            obs_front = True
                        elif psi > 0.2:
                            obs_right = True
                        elif psi < -0.2:
                            obs_left = True

        # prev_vehicle_state = self.vehicle_state
        if self.lane_state == LaneInfo.LEFT_LANE:
            if front_dist <= self.detect_dist and obs_front:
                if not obs_right:
                    self.vehicle_state = "turn-right"
                else:
                    self.vehicle_state = "stop"
            else:
                self.vehicle_state = "straight"

        elif self.lane_state == LaneInfo.RIGHT_LANE:
            if front_dist <= self.detect_dist and obs_front:
                if not obs_left:
                    self.vehicle_state = "turn-left"
                else:
                    self.vehicle_state = "stop"
            else:
                self.vehicle_state = "straight"

        elif self.lane_state == LaneInfo.CENTER_LANE:
            if front_dist > self.detect_dist:
                self.vehicle_state = "straight"
            else:
                if not obs_front:
                    self.vehicle_state = "straight"
                elif not obs_right:
                    self.vehicle_state = "turn-right"
                elif not obs_left:
                    self.vehicle_state = "turn-left"
                else:
                    self.vehicle_state = "stop"

        if self.vehicle_state == "stop":
            self.speed = 5
        else:
            self.speed = 20

        # print(front_dist, self.lane_state, self.vehicle_state, obs_front, obs_left, obs_right)

        while not self.target_x or not self.target_y:
            continue

        distToTargetX = abs(self.target_x - curr_x)
        distToTargetY = abs(self.target_y - curr_y)

        if ((distToTargetX < 5 and distToTargetY < 5)):

            prev_target_x = self.target_x
            prev_target_y = self.target_y

            self.target_x = self.lane_marker.x
            self.target_y = self.lane_marker.y

            target_orientation = np.arctan2(self.target_y - prev_target_y,
                                            self.target_x - prev_target_x)

            if self.vehicle_state == "turn-right":
                # self.change_lane = False
                tmp_x = 6
                tmp_y = 0
                x_offset = np.cos(target_orientation + np.pi /
                                  2) * tmp_x - np.sin(target_orientation +
                                                      np.pi / 2) * tmp_y
                y_offset = np.sin(target_orientation + np.pi /
                                  2) * tmp_x + np.cos(target_orientation +
                                                      np.pi / 2) * tmp_y
                self.target_x = self.target_x + x_offset
                self.target_y = self.target_y + y_offset
            elif self.vehicle_state == "turn-left":
                # self.change_lane = False
                tmp_x = 6
                tmp_y = 0
                x_offset = np.cos(target_orientation - np.pi /
                                  2) * tmp_x - np.sin(target_orientation -
                                                      np.pi / 2) * tmp_y
                y_offset = np.sin(target_orientation - np.pi /
                                  2) * tmp_x + np.cos(target_orientation -
                                                      np.pi / 2) * tmp_y
                self.target_x = self.target_x + x_offset
                self.target_y = self.target_y + y_offset

        else:
            self.counter += 1

        return [self.target_x, self.target_y, self.speed]


class VehicleController():
    def stop(self):
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.acceleration = -20
        newAckermannCmd.speed = 0
        newAckermannCmd.steering_angle = 0
        return newAckermannCmd

    def execute(self, currentPose, targetPose):
        """
            This function takes the current state of the vehicle and
            the target state to compute low-level control input to the vehicle
            Inputs:
                currentPose: ModelState, the current state of vehicle
                targetPose: The desired state of the vehicle
        """

        currentEuler = currentPose[1]
        curr_x = currentPose[0][0]
        curr_y = currentPose[0][1]

        target_x = targetPose[0]
        target_y = targetPose[1]
        target_v = targetPose[2]

        k_s = 0.1
        k_ds = 1
        k_n = 0.1
        k_theta = 1

        # compute errors
        dx = target_x - curr_x
        dy = target_y - curr_y
        xError = (target_x - curr_x) * np.cos(
            currentEuler[2]) + (target_y - curr_y) * np.sin(currentEuler[2])
        yError = -(target_x - curr_x) * np.sin(
            currentEuler[2]) + (target_y - curr_y) * np.cos(currentEuler[2])
        curr_v = np.sqrt(currentPose[2][0]**2 + currentPose[2][1]**2)
        vError = target_v - curr_v

        delta = k_n * yError
        # Checking if the vehicle need to stop
        if target_v > 0:
            v = xError * k_s + vError * k_ds
            #Send computed control input to vehicle
            newAckermannCmd = AckermannDrive()
            newAckermannCmd.speed = v
            newAckermannCmd.steering_angle = delta
            return newAckermannCmd
        else:
            return self.stop()


class Controller(object):
    """docstring for Controller"""
    def __init__(self):
        super(Controller, self).__init__()
        self.decisionModule = VehicleDecision()
        self.controlModule = VehicleController()

    def stop(self):
        return self.controlModule.stop()

    def execute(self, currState, obstacleList, lane_marker, waypoint):
        # Get the target state from decision module
        refState = self.decisionModule.get_ref_state(currState, obstacleList,
                                                     lane_marker, waypoint)
        if not refState:
            return self.controlModule.stop()
        return self.controlModule.execute(currState, refState)
