import pickle
import numpy as np
from util.util import quaternion_to_euler

class VehicleDecision():
    def __init__(self, fn):
        self.waypoint_list = pickle.load(open(fn,'rb')) # a list of waypoints
        self.pos_idx = int(50)
        self.prev_pos_idx = int(0)

        self.vehicle_state = 'middle'
        self.counter = 0
        self.target_x = self.waypoint_list[self.pos_idx][0]
        self.target_y = self.waypoint_list[self.pos_idx][1]
        self.change_lane = False
        self.change_lane_wp_idx = 0

        
    def get_ref_state(self, currState, obstacleList):
        """
            Get the reference state for the vehicle according to the current state and result from perception module
            Inputs: 
                currState: ModelState, the current state of vehicle
                perceptionInput: float, currently the distance between vehicle and obstacle in front
            Outputs: reference state position and velocity of the vehicle 
        """

        curr_x = currState[0][0]
        curr_y = currState[0][1]

        # Check whether any obstacles are in the front of the vehicle
        # obFlag = False
        obs_front_left = False
        obs_front_right = False
        front_dist = 20
        if obstacleList:
            for obs in obstacleList:
                dy = obs.location.y - curr_y
                dx = obs.location.x - curr_x
                yaw = currState[1][2]
                rx = np.cos(-yaw) * dx - np.sin(-yaw) * dy 
                ry = np.cos(-yaw) * dy + np.sin(-yaw) * dx

                psi = np.arctan(ry/rx)
                if rx > 0:
                    front_dist = np.sqrt(dy*dy + dx*dx)
                    if psi > 0.463:
                        obs_front_right = True

                    if psi < -0.463:
                        obs_front_left = True
                

        prev_vehicle_state = self.vehicle_state          
        if self.vehicle_state == "left":
            if front_dist > 15 and not obs_front_right and not self.change_lane:
                self.vehicle_state = "middle"
            elif front_dist < 15 and not self.change_lane:
                self.vehicle_state = "stop"
        elif self.vehicle_state == "right":
            if front_dist > 15 and not obs_front_left and not self.change_lane:
                self.vehicle_state = "middle"
            elif front_dist < 15 and not self.change_lane:
                self.vehicle_state = "stop"
        elif self.vehicle_state == "middle":
            if front_dist > 15:
                self.vehicle_state = "middle"
            elif not obs_front_left:
                self.vehicle_state = "left"
            elif not obs_front_right:
                self.vehicle_state = "right"
            else:
                self.vehicle_state = "stop"
        else:
            if front_dist > 15:
                self.vehicle_state = "middle"
            elif not obs_front_left:
                self.vehicle_state = "left"
            elif not obs_front_right:
                self.vehicle_state = "right"
        
        if prev_vehicle_state != self.vehicle_state:
            self.change_lane = True
            self.change_lane_wp_idx = self.pos_idx

        # self.vehicle_state = "left"
        print(front_dist, self.vehicle_state, obs_front_left, obs_front_right, self.change_lane)
        target_x = self.target_x
        target_y = self.target_y

        # curr_x = currState.pose.position.x
        # curr_y = currState.pose.position.y
        
        distToTargetX = abs(target_x - curr_x)
        distToTargetY = abs(target_y - curr_y)

        if ((distToTargetX < 4 and distToTargetY < 4)) or self.counter > 100:
            self.counter = 0
            prev_target_x = self.waypoint_list[self.pos_idx][0]
            prev_target_y = self.waypoint_list[self.pos_idx][1]

            if self.change_lane:
                self.pos_idx = self.pos_idx + 100
            else:
                self.pos_idx = self.pos_idx + 50
            # self.pos_idx = self.pos_idx % len(self.waypoint_list)
            if self.pos_idx > self.change_lane_wp_idx + 150:
                self.change_lane = False

            if self.pos_idx > len(self.waypoint_list):
                return None

            self.target_x = self.waypoint_list[self.pos_idx][0]
            self.target_y = self.waypoint_list[self.pos_idx][1]

            target_orientation = np.arctan2(self.target_y-prev_target_y, 
                self.target_x-prev_target_x)
            if self.vehicle_state == "right":
                tmp_x = 3
                tmp_y = 0
                x_offset = np.cos(target_orientation+np.pi/2)*tmp_x - np.sin(target_orientation+np.pi/2)*tmp_y
                y_offset = np.sin(target_orientation+np.pi/2)*tmp_x + np.cos(target_orientation+np.pi/2)*tmp_y
                self.target_x = self.target_x + x_offset
                self.target_y = self.target_y + y_offset
            elif self.vehicle_state == "left":
                tmp_x = 3
                tmp_y = 0
                x_offset = np.cos(target_orientation-np.pi/2)*tmp_x - np.sin(target_orientation-np.pi/2)*tmp_y
                y_offset = np.sin(target_orientation-np.pi/2)*tmp_x + np.cos(target_orientation-np.pi/2)*tmp_y
                self.target_x = self.target_x + x_offset
                self.target_y = self.target_y + y_offset

            print("reached",self.waypoint_list[self.pos_idx-20][0],self.waypoint_list[self.pos_idx-20][1],
                "next",self.waypoint_list[self.pos_idx][0],self.waypoint_list[self.pos_idx][1])
        else:
            self.counter += 1

        return [target_x, target_y, 0, 6]

