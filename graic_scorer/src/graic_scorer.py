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

collisionPenalty = 40
deviationPenalty = 10


class EvaluationNode:
    def __init__(self,
                 world,
                 role_name='ego_vehicle',
                 track='t1_triple',
                 reset=True):
        self.subCollision = rospy.Subscriber('/carla/%s/collision' % role_name,
                                             CarlaCollisionEvent,
                                             self.collisionCallback)
        self.subLocation = rospy.Subscriber('/carla/%s/location' % role_name,
                                            LocationInfo,
                                            self.locationCallback)
        self.subWaypoint = rospy.Subscriber('/carla/%s/waypoints' % role_name,
                                            WaypointInfo,
                                            self.waypointCallback)
        self.subLaneInvasion = rospy.Subscriber(
            '/carla/%s/lane_invasion' % role_name, CarlaLaneInvasionEvent,
            self.laneCallback)
        self.pubReach = rospy.Publisher('/carla/%s/reached' % role_name,
                                        String,
                                        queue_size=None)
        self.pubScore = rospy.Publisher('/carla/%s/score' % role_name,
                                        Float32,
                                        queue_size=None)
        self.pubCollision = rospy.Publisher('/carla/%s/collision_detail' %
                                            role_name,
                                            String,
                                            queue_size=None)
        self.reachedPoints = []
        self.reachedPointsStamped = []
        # self.speedList = []
        self.reachedTime = 0
        self.hitObjects = set()
        self.score = 0.0
        self.location = None
        self.role_name = role_name
        self.waypoint = None
        self.reachEnd = False
        self.obs_map = {}
        self.world = world
        self.map = self.world.get_map()
        self.trajectory_list = []
        self.deviated = False
        self.reset = reset
        self.addActor()
        self.vehicle = None
        while not self.vehicle:
            self.find_ego_vehicle()

    def find_ego_vehicle(self):
        for actor in self.world.get_actors():
            if actor.attributes.get('role_name') == self.role_name:
                self.vehicle = actor
                break

    def addActor(self):
        actor_list = self.world.get_actors()
        env_list = self.world.get_environment_objects()

        for actor in actor_list:
            self.obs_map[str(
                actor.id)] = str(actor.type_id) + '_' + str(actor.id)
        self.obs_map['0'] = 'fence'

    def locationCallback(self, data):
        self.location = data

    def collisionCallback(self, data):
        if not self.obs_map:
            return

        if self.reachEnd:
            return

        if str(data.other_actor_id) == '0':
            return

        if not str(data.other_actor_id) in self.obs_map:
            self.obs_map[str(data.other_actor_id)] = str(
                self.world.get_actor(data.other_actor_id).type_id) + '_' + str(
                    data.other_actor_id)

        hitObj = self.obs_map[str(data.other_actor_id)] + "_at_time_" + str(
            datetime.timedelta(seconds=int(rospy.get_rostime().to_sec())))

        if str(data.other_actor_id) != '0' and self.reset:
            print("REACH COLLISION")
            transform = self.map.get_waypoint(
                self.vehicle.get_location() -
                15 * self.vehicle.get_transform().get_forward_vector(),
                project_to_road=True,
                lane_type=carla.LaneType.Driving).transform
            transform.location.z += 1
            transform.rotation.roll = 0
            transform.rotation.pitch = 0
            self.vehicle.set_transform(transform)
            self.vehicle.set_target_angular_velocity(carla.Vector3D(x=0, y=0))

        if hitObj in self.hitObjects:
            return

        self.hitObjects.add(hitObj)
        rospy.loginfo("Collision with {}".format(self.obs_map[str(
            data.other_actor_id)]))
        collisionInfo = String()
        collisionInfo.data = hitObj
        self.pubCollision.publish(collisionInfo)
        self.score += collisionPenalty

    def waypointCallback(self, data):
        self.waypoint = data.location
        self.reachEnd = data.reachedFinal

    def laneCallback(self, data):
        if self.reachEnd:
            return

        for marking in data.crossed_lane_markings:
            if marking is CarlaLaneInvasionEvent.LANE_MARKING_SOLID and not self.deviated:
                # self.deviated = True
                self.score += deviationPenalty
                return

                # Reset 
                print("Lane Marking")
                transform = self.map.get_waypoint(
                    self.vehicle.get_location() -
                    15 * self.vehicle.get_transform().get_forward_vector(),
                    project_to_road=True,
                    lane_type=carla.LaneType.Driving).transform
                transform.location.z = 6
                transform.rotation.roll = 0
                transform.rotation.pitch = 0
                self.vehicle.set_transform(transform)
                self.vehicle.set_target_angular_velocity(carla.Vector3D(x=0, y=0))


    def calculateScore(self):
        location = self.location
        waypoint = self.waypoint
        if not location or not waypoint:
            return
        x = location.location.x
        y = location.location.y

        distanceToX = abs(x - waypoint.x)
        distanceToY = abs(y - waypoint.y)

        # NOTE reached function; range
        if distanceToX < 8 and distanceToY < 8 and not (
                waypoint.x, waypoint.y) in self.reachedPoints:
            # Reach information
            reached = String()
            reachInfo = "({} reached {:.2f}, {:.2f}) at time {}".format(
                self.role_name, waypoint.x, waypoint.y,
                str(
                    datetime.timedelta(
                        seconds=int(rospy.get_rostime().to_sec()))))
            reached.data = reachInfo

            self.pubReach.publish(reached)
            self.reachedPoints.append((waypoint.x, waypoint.y))
            self.reachedPointsStamped.append(reachInfo)

            if self.reachedTime != 0:
                self.score += int(
                    rospy.get_rostime().to_sec()) - self.reachedTime
            self.reachedTime = int(rospy.get_rostime().to_sec())

    def onShutdown(self):
        fname = 'score_{}.txt'.format(
            self.role_name)
        # fname_trajectory = 'trajectory_{}_{}'.format(self.role_name, time.asctime().replace(' ', '_').replace(':', '_'))
        if not self.reachEnd:
            self.score = 'infinity'
        rospy.loginfo("Final score: {}".format(self.score))
        # fname = 'score_h'
        # print("hit: ", self.hitObjects)
        f = open(fname, 'wb')
        f.write("Final score: \n".encode('ascii'))
        f.write(str(self.score).encode('ascii') + "\n".encode('ascii'))
        f.write("Obstacle hits: \n".encode('ascii'))
        f.write('\n'.join(self.hitObjects).encode('ascii'))
        f.write("\nWaypoints reached: \n".encode('ascii'))
        f.write('\n'.join(self.reachedPointsStamped).encode('ascii'))

        f.close()

        # pickle.dump(self.trajectory_list, open(fname_trajectory, 'wb+'))
        # f = open(fname_trajectory, 'w+')
        # for i in range(len(self.trajectory_list)):
        #     f.write(str(self.trajectory_list[i])+'\n')
        # f.close()


def run(en, role_name):
    rate = rospy.Rate(20)  # 20 Hz
    rospy.on_shutdown(en.onShutdown)
    pubEN = rospy.Publisher('/carla/%s/evaluation' % role_name,
                            EvaluationInfo,
                            queue_size=None)
    while not rospy.is_shutdown():
        en.calculateScore()
        info = EvaluationInfo()
        info.score = en.score
        info.numObjectsHit = len(en.hitObjects)
        pubEN.publish(info)
        score_ = Float32()
        score_.data = float(en.score)
        en.pubScore.publish(score_)
        rate.sleep()

        # Get all actors
        actor_list = en.world.get_actors()
        actors_state = []
        for actor in actor_list:
            if 'vehicle' in actor.type_id:
                actor_transform = actor.get_transform()
                tmp = [
                    actor_transform.location.x,
                    actor_transform.location.y,
                    actor_transform.location.z,
                    actor_transform.rotation.roll,
                    actor_transform.rotation.pitch,
                    actor_transform.rotation.yaw,
                ]
                actors_state += tmp

        en.trajectory_list.append(actors_state)


if __name__ == "__main__":
    rospy.init_node("Evaluation_Node")
    host = rospy.get_param('~host', 'localhost')
    port = rospy.get_param('~port', 2000)
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    track = rospy.get_param("~track", "t1_triple")
    model_free = rospy.get_param("~model_free", 1)
    rospy.loginfo("Start evaluating the performance of %s!" % role_name)
    os.chdir(os.path.dirname(__file__))
    cwd = os.getcwd()
    client = carla.Client(host, port)
    world = client.get_world()
    if model_free:
        reset = True
    else:
        reset = False
    en = EvaluationNode(world, role_name, track, reset)
    try:
        run(en, role_name)
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down evaluation node")
