import rospy 
import numpy as np
from carla_msgs.msg import CarlaCollisionEvent
from popgri_msgs.msg import LocationInfo, EvaluationInfo
import time
import pickle
import os

class EvaluationNode:

    def __init__(self, role_name='ego_vehicle'):
        self.subCollision = rospy.Subscriber('/carla/%s/collision'%role_name, CarlaCollisionEvent, self.collisionCallback)
        self.subLocation = rospy.Subscriber('/carla/%s/location'%role_name, LocationInfo, self.locationCallback)
        self.waypoint_list = pickle.load(open('waypoints','rb'))
        self.reachedPoints = []
        self.speedList = []
        self.hitObjects = set()
        self.deviationCount = 0
        self.score = 0.0
        self.location = None
        self.role_name = role_name

    def locationCallback(self, data):
        self.location = data

    def collisionCallback(self, data):
        if str(data.other_actor_id) in self.hitObjects:
            return
        # print("Collision Detected with ", data.objects_hit[0])
        self.hitObjects.add(str(data.other_actor_id))
        self.score -= 100.0

    def calculateScore(self):
        location = self.location
        if not location:
            return
        x = location.location.x
        y = location.location.y 
        closedIdx = 0
        closed = [0, 0, 10000]

        for idx, waypoint in enumerate(self.waypoint_list):
            wx = waypoint[0]
            wy = waypoint[1]
            dist = np.sqrt((x-wx)*(x-wx) + (y-wy)*(y-wy))
            if dist < closed[2]:
                closed = [wx, wy, dist]
                closedIdx = idx
        
        distanceToX = abs(x - closed[0])
        distanceToY = abs(y - closed[1])


        vx = location.velocity.x
        vy = location.velocity.y
        v = np.sqrt(vx*vx + vy*vy)
        if v > 0.5:
            self.speedList.append(v)
        
        if distanceToX < 4 and distanceToY < 4 and [closed[0], closed[1]] not in self.reachedPoints:
            self.reachedPoints.append([closed[0], closed[1]])
            # print("Reached ", closed[0], closed[1])
            vBar = np.average(self.speedList)
            if np.isnan(vBar):
                return
            self.speedList = []
            self.score += vBar
            # print("Current Score: ", self.score)

        # return self.score, self.hitObjects

    def onShutdown(self):
        fname = 'score_{}_{}'.format(self.role_name, time.asctime())
        # fname = 'score_h'
        f = open(fname, 'wb')
        f.write(str(self.score).encode('ascii'))
        f.write('\n')
        f.write(str(self.hitObjects).encode('ascii'))
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
        pubEN.publish()
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("Evaluation_Node")
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    print("Start evaluating the performance of %s!"%role_name)
    os.chdir(os.path.dirname(__file__))
    cwd = os.getcwd()
    en = EvaluationNode(role_name=role_name)
    run(en, role_name)

