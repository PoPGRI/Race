import rospy 
import numpy as np
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int16
import time
import pickle
import os

class WaypointNode:

    def __init__(self, role_name='ego_vehicle'):
        self.subReach = rospy.Subscriber('/carla/%s/reached'%role_name, Int16, self.reachCallback)
        # self.pubWaypoint = rospy.Subscriber('/carla/%s/waypoints'%role_name, Vector3, queue_size=1)
        self.waypoint_list = pickle.load(open('waypoints','rb'))
        self.role_name = role_name

    def getWaypoint(self):
        if len(self.waypoint_list) != 0:
            return self.waypoint_list[0]

    def reachCallback(self, data):
        for _ in range(50):
            if len(self.waypoint_list) != 0:
                self.waypoint_list.pop(0)

def run(wn, role_name):
    rate = rospy.Rate(100)  # 100 Hz    
    pubWaypoint = rospy.Publisher('/carla/%s/waypoints'%role_name, Vector3, queue_size=1)
    while not rospy.is_shutdown():
        waypoint = wn.getWaypoint()
        pub_waypoint = Vector3()
        pub_waypoint.x = waypoint[0]
        pub_waypoint.y = waypoint[1]
        pubWaypoint.publish(pub_waypoint)
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("Evaluation_Node")
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    rospy.loginfo("Start evaluating the performance of %s!"%role_name)
    os.chdir(os.path.dirname(__file__))
    cwd = os.getcwd()
    wn = WaypointNode(role_name=role_name)
    run(wn, role_name)

