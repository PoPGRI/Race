import rospy 
import numpy as np
from carla_msgs.msg import CarlaCollisionEvent
from carla_msgs.msg import CarlaLaneInvasionEvent
from graic_msgs.msg import LocationInfo, EvaluationInfo, WaypointInfo
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int16, Float32, String
import time
import pickle
import carla
import os
import datetime

collisionPenalty = 60
deviationPenalty = 30
t1Factor = 900
t2Factor = 1600

class EvaluationNode:

    def __init__(self, world, role_name='ego_vehicle', track='t1_triple'):
        self.subCollision = rospy.Subscriber('/carla/%s/collision'%role_name, CarlaCollisionEvent, self.collisionCallback)
        self.subLocation = rospy.Subscriber('/carla/%s/location'%role_name, LocationInfo, self.locationCallback)
        self.subWaypoint = rospy.Subscriber('/carla/%s/waypoints'%role_name, WaypointInfo, self.waypointCallback)
        self.subLaneInvasion = rospy.Subscriber('carla/%s/lane_invasion'%role_name, CarlaLaneInvasionEvent, self.laneCallback)
        self.pubReach = rospy.Publisher('/carla/%s/reached'%role_name, String, queue_size=1)
        self.pubScore = rospy.Publisher('/carla/%s/score'%role_name, Float32, queue_size=1)
        self.pubCollision = rospy.Publisher('/carla/%s/collision_detail'%role_name, String, queue_size=1)
        self.reachedPoints = []
        self.reachedPointsStamped = []
        self.speedList = []
        self.hitObjects = set()
        self.score = 0.0
        self.location = None
        self.role_name = role_name
        self.waypoint = None
        self.reachEnd = False
        self.obs_map = {}
        self.world = world
        if track == 't1_triple':
            self.scoreFactor = t1Factor
        elif track == 't2_triple':
            self.scoreFactor = t2Factor
        self.addActor()

        
    def addActor(self):
        actor_list = self.world.get_actors()
        env_list = self.world.get_environment_objects()
        
        for actor in actor_list:
            self.obs_map[str(actor.id)] = str(actor.type_id) + '_' + str(actor.id)
        self.obs_map['0'] = 'fence'

    def locationCallback(self, data):
        self.location = data

    def collisionCallback(self, data):
        if not self.obs_map:
            return

        if self.reachEnd:
            return 
        
        hitObj = self.obs_map[str(data.other_actor_id)]+"_at_time_"+str(datetime.timedelta(
                seconds=int(rospy.get_rostime().to_sec())))
        if hitObj in self.hitObjects:
            return

        self.hitObjects.add(hitObj)
        rospy.loginfo("Collision with {}".format(self.obs_map[str(data.other_actor_id)]))
        collisionInfo = String()
        collisionInfo.data = hitObj
        self.pubCollision.publish(collisionInfo)
        self.score -= collisionPenalty

    def waypointCallback(self, data):
        self.waypoint = data.location
        self.reachEnd = data.reachedFinal

    def laneCallback(self, data):
        if self.reachEnd:
            return 
        
        for marking in data.crossed_lane_markings:
            if marking is CarlaLaneInvasionEvent.LANE_MARKING_SOLID:
                self.score -= deviationPenalty

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
        if distanceToX < 8 and distanceToY < 8 and not (waypoint.x, waypoint.y) in self.reachedPoints: 
            # Reach information
            reached = String()
            reachInfo = "({} reached {:.2f}, {:.2f}) at time {}".format(self.role_name, waypoint.x, waypoint.y, str(datetime.timedelta(
                seconds=int(rospy.get_rostime().to_sec()))))
            reached.data = reachInfo
            self.pubReach.publish(reached)
            self.reachedPoints.append((waypoint.x, waypoint.y))
            self.reachedPointsStamped.append(reachInfo)

            vBar = np.average(self.speedList)
            if np.isnan(vBar):
                return
            self.speedList = []
            self.score += vBar

    def onShutdown(self):
        fname = 'score_{}_{}'.format(self.role_name, time.asctime().replace(' ', '_').replace(':', '_'))
        rospy.loginfo("Final score: {}".format(self.score))
        # fname = 'score_h'
        # print("hit: ", self.hitObjects)
        f = open(fname, 'wb')
        f.write("Final score: \n".encode('ascii'))
        f.write(str(self.score/self.scoreFactor*100).encode('ascii') + "\n".encode('ascii'))
        f.write("Obstacle hits: \n".encode('ascii'))
        f.write('\n'.join(self.hitObjects).encode('ascii'))
        f.write("\nWaypoints reached: \n".encode('ascii'))
        f.write('\n'.join(self.reachedPointsStamped).encode('ascii'))
        f.close()


def run(en, role_name):
    rate = rospy.Rate(20)  # 20 Hz    
    rospy.on_shutdown(en.onShutdown)
    pubEN = rospy.Publisher('/carla/%s/evaluation'%role_name, EvaluationInfo, queue_size=1)
    while not rospy.is_shutdown():
        en.calculateScore()
        info = EvaluationInfo()
        info.score = en.score 
        info.numObjectsHit = len(en.hitObjects)
        pubEN.publish(info)
        score_ = Float32()
        score_.data = float(en.score/en.scoreFactor*100)
        en.pubScore.publish(score_)
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("Evaluation_Node")
    host = rospy.get_param('~host', 'localhost')
    port = rospy.get_param('~port', 2000)
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    track = rospy.get_param("~track", "t1_triple")
    rospy.loginfo("Start evaluating the performance of %s!"%role_name)
    os.chdir(os.path.dirname(__file__))
    cwd = os.getcwd()
    client = carla.Client(host, port)
    world = client.get_world()
    en = EvaluationNode(world, role_name, track)
    try:
        run(en, role_name)
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down the evaluation node")

