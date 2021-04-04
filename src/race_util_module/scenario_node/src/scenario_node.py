import rospy 
import numpy as np
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
from graic_msgs.msg import WaypointInfo
import time
import pickle
import carla
import os


class ScenarioGenerator:
    scenarios = []
    def __init__(self, ego_vehicle, role_name, waypoints):
        pass 

    def createScenario(self, ego_vehicle_status, result=None):
        pass

class ScenarioNode:

    def __init__(self, world, role_name='ego_vehicle', track='t1_tripe'):
        self.subReach = rospy.Subscriber('/carla/%s/reached'%role_name, String, self.reachCallback)
        # self.waypoint_list = pickle.load(open(track,'rb')) 
        self.role_name = role_name
        self.world = world
        self.map = world.get_map()
        self.ego_vehicle = None
        self.scenarioGenerator = ScenarioGenerator()

        self.findEgoVehicle()

    def findEgoVehicle(self):
        for actor in self.world.get_actors():
            if actor.attributes.get('role_name') == self.role_name:
                self.ego_vehicle = actor
                break

    def getVehicleStatus(self):
        if not self.ego_vehicle:
            self.findEgoVehicle
        return self.findEgoVehicle.get_transform()

    def createScenarioConfig(self, result=None):
        self.scenarioGenerator.createScenario(self.getVehicleStatus(), result)

    def launchScenarioRunner(self, scenarioConfig):
        # TODO 
        pass 

    def parsingResults(self, results):
        pass 

def run(sn, role_name):
    # rate = rospy.Rate(20)  # 20 Hz    
    # pubWaypoint = rospy.Publisher('/carla/%s/waypoints'%role_name, WaypointInfo, queue_size=None)
    # Send out all unit scenariose
    scenarioConfig = sn.createScenarioConfig()
    firstResults = sn.launchScenarioRunner(scenarioConfig)
    sn.parseResults(firstResults)
    scenarioConfig = sn.createScenarioConfig(firstResults)
    secondResults = sn.launchScenarioRunner(scenarioConfig)
    sn.parseResults(secondResults)

    # NOTE Question: Do we need to send out scores often? 
    # while not rospy.is_shutdown():
    #     rate.sleep()

if __name__ == "__main__":
    rospy.init_node("Scenario_Node")
    host = rospy.get_param('~host', 'localhost')
    port = rospy.get_param('~port', 2000)
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    track = rospy.get_param("~track", "t1_triple")
    rospy.loginfo("Start generating scenarios for %s!"%role_name)
    os.chdir(os.path.dirname(__file__))
    cwd = os.getcwd()
    client = carla.Client(host, port)
    world = client.get_world()
    sn = ScenarioNode(world, role_name, track)
    try:
        run(sn, role_name)
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down waypoint node")

