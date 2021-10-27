import rospy 
import rospkg
import numpy as np
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
from graic_msgs.msg import WaypointInfo
import time
import pickle
import carla
import os

class WaypointNode:

    def __init__(self, world, role_name='ego_vehicle', track='t1_tripe'):
        self.subReach = rospy.Subscriber('/carla/%s/reached'%role_name, String, self.reachCallback)
        rospack = rospkg.RosPack()
        fpath = rospack.get_path('config_node')
        self.waypoint_list = pickle.load(open(fpath+'/'+track,'rb'))
        self.waypoint_list.pop(0)
        self.role_name = role_name
        self.world = world
        self.map = world.get_map()

    def getWaypoint(self):
        if len(self.waypoint_list) != 0:
            location = carla.Location(self.waypoint_list[0][0], self.waypoint_list[0][1], self.waypoint_list[0][2])
            rotation = self.map.get_waypoint(location, project_to_road=True, lane_type=carla.LaneType.Driving).transform.rotation
            box = carla.BoundingBox(location, carla.Vector3D(0,6,3))
            if len(self.waypoint_list) == 1:
                self.world.debug.draw_box(box, rotation, thickness=0.5, color=carla.Color(255, 255, 0, 255), life_time=0)
            else:
                self.world.debug.draw_box(box, rotation, thickness=0.5, color=carla.Color(0, 255, 0, 255), life_time=2)
            return self.waypoint_list[0]
        else:
            return None
    
    def isFinal(self):
        return len(self.waypoint_list) == 1
    
    # TODO change this for new map
    def reachCallback(self, data):
        if len(self.waypoint_list) > 0:
            self.waypoint_list.pop(0)


def run(wn, role_name):
    rate = rospy.Rate(20)  # 20 Hz    
    pubWaypoint = rospy.Publisher('/carla/%s/waypoints'%role_name, WaypointInfo, queue_size=None)
    while not rospy.is_shutdown():
        waypoint = wn.getWaypoint()
        pub_waypoint = WaypointInfo()
        pub_waypoint.role_name = role_name
        if not waypoint:
            pub_waypoint.reachedFinal = True
        else:
            pub_waypoint.location.x = waypoint[0]
            pub_waypoint.location.y = waypoint[1]
            pub_waypoint.location.z = waypoint[2]
        pubWaypoint.publish(pub_waypoint)
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("Waypoint_Node")
    host = rospy.get_param('~host', 'localhost')
    port = rospy.get_param('~port', 2000)
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    track = rospy.get_param("~track", "t1_triple")
    rospy.loginfo("Start publishing waypoints for %s!"%role_name)
    os.chdir(os.path.dirname(__file__))
    cwd = os.getcwd()
    client = carla.Client(host, port)
    world = client.get_world()
    wn = WaypointNode(world, role_name, track)
    try:
        run(wn, role_name)
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down waypoint node")

