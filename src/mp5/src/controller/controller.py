import rospy
from ackermann_msgs.msg import AckermannDrive
import numpy as np
from util.util import euler_to_quaternion, quaternion_to_euler

class VehicleController():

    def __init__(self, model_name='gem'):
        # Publisher to publish the control input to the vehicle model
        self.controlPub = rospy.Publisher("/" + model_name + "/ackermann_cmd", AckermannDrive, queue_size = 1)
        self.model_name = model_name

    def execute(self, currentPose, targetPose):
        """
            This function takes the current state of the vehicle and 
            the target state to compute low-level control input to the vehicle
            Inputs: 
                currentPose: ModelState, the current state of vehicle
                targetPose: The desired state of the vehicle
        """

        currentEuler = quaternion_to_euler(currentPose.pose.orientation.x,
                                           currentPose.pose.orientation.y,
                                           currentPose.pose.orientation.z,
                                           currentPose.pose.orientation.w)
        
        target_x = targetPose[0]
        target_y = targetPose[1]
        target_v = targetPose[2]
        
        k_s = 0.1
        k_ds = 1
        k_n = 0.1

        #compute errors
        xError = (target_x - currentPose.pose.position.x) * np.cos(currentEuler[2]) + (target_y - currentPose.pose.position.y) * np.sin(currentEuler[2])
        yError = -(target_x - currentPose.pose.position.x) * np.sin(currentEuler[2]) + (target_y - currentPose.pose.position.y) * np.cos(currentEuler[2])
        curr_v = np.sqrt(currentPose.twist.linear.x**2 + currentPose.twist.linear.y**2)
        vError = target_v - curr_v
        
        delta = k_n*yError
        # Checking if the vehicle need to stop
        if target_v > 0:
            v = xError*k_s + vError*k_ds
        else:
            v = xError*k_s - 0.05*k_ds            

        #Send computed control input to vehicle
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = v
        newAckermannCmd.steering_angle = delta
        self.controlPub.publish(newAckermannCmd)

