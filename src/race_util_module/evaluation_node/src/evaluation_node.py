import rospy 
import numpy as np
from carla_msgs.msg import CarlaCollisionEvent
from popgri_msgs.msg import LocationInfo, EvaluationInfo
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int16
import time
import pickle
import os

class EvaluationNode:

    def __init__(self, role_name='ego_vehicle'):
        self.subCollision = rospy.Subscriber('/carla/%s/collision'%role_name, CarlaCollisionEvent, self.collisionCallback)
        self.subLocation = rospy.Subscriber('/carla/%s/location'%role_name, LocationInfo, self.locationCallback)
        self.subWaypoint = rospy.Subscriber('/carla/%s/waypoints'%role_name, Vector3, self.waypointCallback)
        self.pubReach = rospy.Publisher('/carla/%s/reached'%role_name, Int16, queue_size=1)
        # self.waypoint_list = pickle.load(open('waypoints','rb'))
        self.reachedPoints = []
        self.speedList = []
        self.hitObjects = set()
        self.deviationCount = 0
        self.score = 0.0
        self.location = None
        self.role_name = role_name
        self.waypoint = None

    def locationCallback(self, data):
        self.location = data

    def collisionCallback(self, data):
        if str(data.other_actor_id) in self.hitObjects:
            return
        # print("Collision Detected with ", data.other_actor_id)
        self.hitObjects.add(str(data.other_actor_id)+"_at_time_"+str(data.header.stamp))
        self.score -= 100.0

    def waypointCallback(self, data):
        self.waypoint = data

    def calculateScore(self):
        location = self.location
        waypoint = self.waypoint
        if not location or not waypoint:
            return
        x = location.location.x
        y = location.location.y 
        
        distanceToX = abs(x - waypoint.x)
        distanceToY = abs(y - waypoint.y)


        vx = location.velocity.x
        vy = location.velocity.y
        v = np.sqrt(vx*vx + vy*vy)
        if v > 0.5:
            self.speedList.append(v)
        
        # NOTE reached function; range
        if distanceToX < 8 and distanceToY < 8: 
            reached = Int16()
            reached.data = 1
            self.pubReach.publish(reached)
            vBar = np.average(self.speedList)
            if np.isnan(vBar):
                return
            self.speedList = []
            self.score += vBar

        # return self.score, self.hitObjects

    def onShutdown(self):
        fname = 'score_{}_{}'.format(self.role_name, time.asctime())
        # fname = 'score_h'
        # print("hit: ", self.hitObjects)
        f = open(fname, 'wb')
        f.write(str(self.score/2500*100).encode('ascii') + "\n".encode('ascii'))
        f.write("obstacle hits: \n".encode('ascii'))
        f.write('\n'.join(self.hitObjects).encode('ascii'))
        f.close()


def run(en, role_name):
    rate = rospy.Rate(100)  # 100 Hz    
    rospy.on_shutdown(en.onShutdown)
    pubEN = rospy.Publisher('/carla/%s/evaluation'%role_name, EvaluationInfo, queue_size=1)
    while not rospy.is_shutdown():
        en.calculateScore()
        info = EvaluationInfo()
        info.score = en.score 
        info.numObjectsHit = len(en.hitObjects)
        pubEN.publish(info)
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("Evaluation_Node")
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    print("Start evaluating the performance of %s!"%role_name)
    os.chdir(os.path.dirname(__file__))
    cwd = os.getcwd()
    en = EvaluationNode(role_name=role_name)
    try:
        run(en, role_name)
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down the evaluation node")

