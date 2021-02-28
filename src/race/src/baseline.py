import rospy
import numpy as np
import argparse
import time
from graic_msgs.msg import ObstacleList, ObstacleInfo
from graic_msgs.msg import LocationInfo
from ackermann_msgs.msg import AckermannDrive
from carla_msgs.msg import CarlaEgoVehicleControl
from graic_msgs.msg import LaneList
from graic_msgs.msg import LaneInfo

class VehicleDecision():
    def __init__(self, role_name):
        self.subWaypoint = rospy.Subscriber("/carla/%s/lane_waypoints"%role_name, LaneList, self.waypointCallback)

        self.vehicle_state = 'straight'
        self.lane_state = 0
        self.counter = 0

        self.waypoint = None
        self.target_x = None 
        self.target_y = None
        self.change_lane = False
        self.change_lane_wp_idx = 0
        self.detect_dist = 15
        self.speed = 20


    def waypointCallback(self, data):
        self.waypoint = data.lane_waypoints[-1]
        self.lane_state = self.waypoint.lane_state
        if not self.target_x or not self.target_y:
            self.target_x = self.waypoint.location.x 
            self.target_y = self.waypoint.location.y

        
    def get_ref_state(self, currState, obstacleList):
        """
            Get the reference state for the vehicle according to the current state and result from perception module
            Inputs: 
                currState: [Loaction, Rotation, Velocity] the current state of vehicle
                obstacleList: List of obstacles
            Outputs: reference state position and velocity of the vehicle 
        """

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

                    psi = np.arctan(ry/rx)
                    if rx > 0:
                        front_dist = np.sqrt(dy*dy + dx*dx)
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
            self.speed = 0
        else:
            self.speed = 20

        print(front_dist, self.lane_state, self.vehicle_state, obs_front, obs_left, obs_right)

        while not self.target_x or not self.target_y:
            continue
        
        distToTargetX = abs(self.target_x - curr_x)
        distToTargetY = abs(self.target_y - curr_y)

        if ((distToTargetX < 5 and distToTargetY < 5)): 
            
            prev_target_x = self.target_x
            prev_target_y = self.target_y

            self.target_x = self.waypoint.location.x 
            self.target_y = self.waypoint.location.y

            target_orientation = np.arctan2(self.target_y-prev_target_y, 
                self.target_x-prev_target_x)
            
            if self.vehicle_state == "turn-right":
                # self.change_lane = False
                tmp_x = 4
                tmp_y = 0
                x_offset = np.cos(target_orientation+np.pi/2)*tmp_x - np.sin(target_orientation+np.pi/2)*tmp_y
                y_offset = np.sin(target_orientation+np.pi/2)*tmp_x + np.cos(target_orientation+np.pi/2)*tmp_y
                self.target_x = self.target_x + x_offset
                self.target_y = self.target_y + y_offset
            elif self.vehicle_state == "turn-left":
                # self.change_lane = False
                tmp_x = 4
                tmp_y = 0
                x_offset = np.cos(target_orientation-np.pi/2)*tmp_x - np.sin(target_orientation-np.pi/2)*tmp_y
                y_offset = np.sin(target_orientation-np.pi/2)*tmp_x + np.cos(target_orientation-np.pi/2)*tmp_y
                self.target_x = self.target_x + x_offset
                self.target_y = self.target_y + y_offset

        else:
            self.counter += 1

        return [self.target_x, self.target_y, self.speed]



class VehicleController():

    def __init__(self, role_name='ego_vehicle'):
        # Publisher to publish the control input to the vehicle model
        self.controlPub = rospy.Publisher("/carla/%s/ackermann_control"%role_name, AckermannDrive, queue_size = 1)

    def stop(self):
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.acceleration = -20
        newAckermannCmd.speed = 0
        newAckermannCmd.steering_angle = 0
        self.controlPub.publish(newAckermannCmd)

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
        xError = (target_x - curr_x) * np.cos(currentEuler[2]) + (target_y - curr_y) * np.sin(currentEuler[2])
        yError = -(target_x - curr_x) * np.sin(currentEuler[2]) + (target_y - curr_y) * np.cos(currentEuler[2])
        curr_v = np.sqrt(currentPose[2][0]**2 + currentPose[2][1]**2)
        vError = target_v - curr_v
        
        delta = k_n*yError 
        # Checking if the vehicle need to stop
        if target_v > 0:
            v = xError*k_s + vError*k_ds
            #Send computed control input to vehicle
            newAckermannCmd = AckermannDrive()
            newAckermannCmd.speed = v
            newAckermannCmd.steering_angle = delta
            self.controlPub.publish(newAckermannCmd)
        else:
            self.stop()           

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


def run_model(role_name):
    
    rate = rospy.Rate(100)  # 100 Hz    

    perceptionModule = VehiclePerception(role_name=role_name)
    decisionModule = VehicleDecision(role_name)
    controlModule = VehicleController(role_name=role_name)

    def shut_down():
        controlModule.stop()
    rospy.on_shutdown(shut_down)

    while not rospy.is_shutdown():
        rate.sleep()  # Wait a while before trying to get a new state
        obstacleList = perceptionModule.obstacleList

        # Get the current position and orientation of the vehicle
        currState =  (perceptionModule.position, perceptionModule.rotation, perceptionModule.velocity)
        if not currState or not currState[0]:
            continue
        
        # Get the target state from decision module
        refState = decisionModule.get_ref_state(currState, obstacleList)
        if not refState:
            controlModule.stop()
            exit(0)

        # Execute 
        controlModule.execute(currState, refState)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Running vechile")

    role_name_default = 'ego_vehicle'

    parser.add_argument('--name', type=str, help='Rolename of the vehicle', default=role_name_default)
    argv = parser.parse_args()
    role_name = argv.name
    rospy.init_node("baseline")
    role_name = 'hero0'
    try:
        run_model(role_name)
    except rospy.exceptions.ROSInterruptException:
        print("stop")
    