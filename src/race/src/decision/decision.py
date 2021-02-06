import pickle
import numpy as np

class VehicleDecision():
    def __init__(self, fn):
        self.waypoint_list = pickle.load(open(fn,'rb')) # a list of waypoints
        self.pos_idx = int(20)
        self.prev_pos_idx = int(0)
        self.counter = 0
        
    def get_ref_state(self, currState):
        """
            Get the reference state for the vehicle according to the current state and result from perception module
            Inputs: 
                currState: ModelState, the current state of vehicle
                perceptionInput: float, currently the distance between vehicle and obstacle in front
            Outputs: reference state position and velocity of the vehicle 
        """

        curr_x = currState[0][0]
        curr_y = currState[0][1]

        target_x = self.waypoint_list[self.pos_idx][0]
        target_y = self.waypoint_list[self.pos_idx][1]

        prev_target_x = self.waypoint_list[self.prev_pos_idx][0]
        prev_target_y = self.waypoint_list[self.prev_pos_idx][1]

        target_orientation = np.arctan2(target_y-prev_target_y, target_x-prev_target_x)
        
        distToTargetX = abs(target_x - curr_x)
        distToTargetY = abs(target_y - curr_y)

        if ((distToTargetX < 2 and distToTargetY < 2)):
            self.prev_pos_idx = self.pos_idx
            self.pos_idx += 10
            if self.pos_idx > len(self.waypoint_list):
                return None
            # self.pos_idx = int(self.pos_idx % len(self.waypoint_list))
            print("reached",self.waypoint_list[self.pos_idx-1][0],self.waypoint_list[self.pos_idx-1][1],
                "next",self.waypoint_list[self.pos_idx][0],self.waypoint_list[self.pos_idx][1])
        

        ref_v = 17

        return [target_x, target_y, target_orientation, ref_v]
