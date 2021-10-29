#!/usr/bin/env python3
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
from carla_msgs.msg import CarlaEgoVehicleControl

import numpy as np
import matplotlib.pyplot as plt
# from matplotlib import animation
from scipy.spatial import ConvexHull, convex_hull_plot_2d

class VehiclePerception:
    def __init__(self, role_name='ego_vehicle'):
        rospy.Subscriber("/carla/%s/location" % role_name, LocationInfo,
                         self.locationCallback)
        rospy.Subscriber("/carla/%s/obstacles" % role_name, ObstacleList,
                         self.obstacleCallback)
        rospy.Subscriber("/carla/%s/lane_markers" % role_name, LaneInfo,
                         self.lanemarkerCallback)
        rospy.Subscriber("/carla/%s/waypoints" % role_name, WaypointInfo,
                         self.waypointCallback)

        self.position = None
        self.rotation = None
        self.velocity = None
        self.obstacleList = None
        self.lane_marker = None
        self.waypoint = None

    def locationCallback(self, data):
        self.position = (data.location.x, data.location.y)
        self.rotation = (np.radians(data.rotation.x),
                         np.radians(data.rotation.y),
                         np.radians(data.rotation.z))
        self.velocity = (data.velocity.x, data.velocity.y)

    def obstacleCallback(self, data):
        self.obstacleList = data.obstacles

    def lanemarkerCallback(self, data):
        self.lane_marker = data

    def waypointCallback(self, data):
        self.waypoint = data

    def ready(self):
        return (self.position
                is not None) and (self.rotation is not None) and (
                    self.velocity
                    is not None) and (self.obstacleList is not None) and (
                        self.lane_marker is not None
                    )  and (self.waypoint is not None)

    def clear(self):
        self.position = None
        self.rotation = None
        self.velocity = None
        self.obstacleList = None
        self.lane_marker = None
        self.waypoint = None

class Visualization2D():
    def __init__(self):
        self.fig = plt.figure()
        self.ax = plt.axes(xlim=(-60, 60), ylim=(-60, 60))
        self.ego_pose = [np.zeros(2, 1), 0.]

    def add_obs(self, obstacleList):
        for ob in obstacleList:
            center = np.array(ob.location)
            self.ax_plot(center[0], center[1], 'ro')
            if len(ob.vertices_locations) > 0:
                assert len(ob.vertices_locations) == 8
                vertices = [v.vertex_location for v in ob.vertices_locations]
                half_level = np.mean([v[2] for v in vertices])
                vertices = np.array([v for v in vertices if v[2] > half_level]).T
                vertices = self.trans(vertices).T[:,:2]
                hull = ConvexHull(vertices)
                self.ax.plot(vertices[hull.vertices,0], vertices[hull.vertices,1], 'r-', lw=2)

    def add_lane(self, lane_markers):
        points = []
        for lane in [lane_markers.lane_markers_center, lane_markers.lane_markers_left, lane_markers.lane_markers_right]:
            points += lane.location
        points = np.array(points).T # 3 x N
        points = self.trans(points)
        self.ax.plot(points[0,:], points[1,:], 'k*', linewidth=1.)

    def add_self(self):
        w = 3; h = 5;
        self.ax.plot([-w/2, -w/2, w/2, w/2, -w/2], [-h/2, h/2, h/2, -h/2, -h/2], 'b-', linewidth=1.)

    def trans(self, points):
        if len(points.shape) == 1:
            p = points.reshape(-1, 1)
        elif len(points.shape) == 2:
            p = points
        else:
            raise ValueError('wrong shape')
        assert p.shape[0] == 2

        rotation = -self.ego_pose[1]
        shift = self.ego_pose[0]
        p = np.array([[np.cos(rotation), -np.sin(rotation)], [np.sin(rotation), np.cos(rotation)]]).matmul(p - shift)

        return p.reshape(points.shape)

    def update(self, pm):
        self.ax.clear()
        self.ego_pose = [pm.position, pm.rotation[-1]]
        self.add_self()
        self.add_lane(pm.lane_marker)
        self.add_obs(pm.obstacleList)

def main(role_name):
    pm = VehiclePerception(role_name=role_name)
    vis = Visualization2D()

    import time

    while not rospy.is_shutdown():
        if pm.ready():
            vis.update(pm)
        time.sleep(0.01)

if __name__ == "__main__":
    rospy.init_node("graic_2d_vis", anonymous=True)
    role_name = rospy.get_param("~role_name", "hero0")
    try:
        main(role_name)
    except rospy.exceptions.ROSInterruptException:
        print("stop")
