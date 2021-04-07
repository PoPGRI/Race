import rospy 
import numpy as np
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
from graic_msgs.msg import WaypointInfo
import time
import pickle
import carla
import os

from srunner.scenarioconfigs.route_scenario_configuration import RouteScenarioConfiguration
from scenario_runner import ScenarioRunner
from typing import List, Dict

class ScenarioArguments:
    def __init__(self, route_config: List[RouteScenarioConfiguration], scenario_config: Dict,
        host = '127.0.0.1', port = '2000', timeout = '10.0', trafficManagerPort = '8000',
        trafficManagerSeed = '0', sync = False, agent = None, agentConfig = '', output = True,
        result_file = False, junit = False, json = False, outputDir = '', configFile = '', 
        additionalScenario = '', debug = False, reloadWorld = False, record = '', 
        randomize = False, repetitions = 1, waitForEgo = False
    ):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.trafficManagerPort = trafficManagerPort
        self.trafficManagerSeed = trafficManagerSeed
        self.sync = sync
        self.agent = agent
        self.agentConfig = agentConfig
        self.output = output
        self.file = result_file
        self.junit = junit
        self.json = json
        self.outputDir = outputDir
        self.configFile = configFile
        self.additionalScenario = additionalScenario
        self.debug = debug
        self.reloadWorld = reloadWorld
        self.record = record
        self.randomize = randomize
        self.repetitions = repetitions
        self.waitForEgo = waitForEgo

        self.route = route_config
        self.scenario_config = scenario_config

class ScenarioGenerator:
    scenarios = []
    def __init__(self, ego_vehicle, role_name, waypoints):
        pass 

    def createScenario(self, ego_vehicle_status, result=None):
        pass

class ScenarioNode:

    def __init__(self, world, role_name='ego_vehicle', track='t1_tripe'):
        # self.subReach = rospy.Subscriber('/carla/%s/reached'%role_name, String, self.reachCallback)
        # self.waypoint_list = pickle.load(open(track,'rb')) 
        self.role_name = role_name
        self.world = world
        self.map = world.get_map()
        self.ego_vehicle = None
        self.scenarioGenerator = ScenarioGenerator(None, None, None)

        self.findEgoVehicle()

    def findEgoVehicle(self):
        for actor in self.world.get_actors():
            if actor.attributes.get('role_name') == self.role_name:
                self.ego_vehicle = actor
                break

    def getVehicleStatus(self):
        if not self.ego_vehicle:
            self.findEgoVehicle
        return self.findEgoVehicle().get_transform()

    def createScenarioConfig(self, result=None):
        # self.scenarioGenerator.createScenario(self.getVehicleStatus(), result)
        if result is None:
            scenario_config = {
                "t1_triple": [
                    {
                        "available_event_configurations": [
                            {
                                "transform": {
                                    "pitch": "0",
                                    "x": "93.1460952758789",
                                    "y": "-1.4191741943359375",
                                    "yaw": "133.83380126953125",
                                    "z": "1.22"
                                }
                            }
                        ],
                        "scenario_type": "ScenarioBM"
                    }
                ]
            }
            
            route_config_list = []
            route_config = RouteScenarioConfiguration()
            route_config.town = 't1_triple'
            route_config.name = 'RouteScenario_0'
            route_config.weather = carla.WeatherParameters(sun_altitude_angle=70)
            route_config.scenario_config = scenario_config
            waypoint_list = []
            waypoint_list.append(carla.Location(
                x=164.0,
                y=-11.0,
                z=0.0
            ))

            waypoint_list.append(carla.Location(
                x=14.7,
                y=69.2,
                z=0.0
            ))

            route_config.trajectory = waypoint_list

            route_config_list.append(route_config)

            args = ScenarioArguments(route_config_list, scenario_config)
        else:
            scenario_config = {
                "t1_triple": [
                    {
                        "available_event_configurations": [
                            {
                                "transform": {
                                    "pitch": "0",
                                    "x": "-85.89",
                                    "y": "27.30",
                                    "yaw": "-98.2802734375",
                                    "z": "1.22"
                                }
                            }
                        ],
                        "scenario_type": "ScenarioBM"
                    }
                ]
            }
            
            route_config_list = []
            route_config = RouteScenarioConfiguration()
            route_config.town = 't1_triple'
            route_config.name = 'RouteScenario_0'
            route_config.weather = carla.WeatherParameters(sun_altitude_angle=70)
            route_config.scenario_config = scenario_config
            waypoint_list = []
            waypoint_list.append(carla.Location(
                x=-23.14,
                y=66.14,
                z=0.0
            ))

            waypoint_list.append(carla.Location(
                x=-74.07,
                y=-90.74,
                z=0.0
            ))

            route_config.trajectory = waypoint_list

            route_config_list.append(route_config)

            args = ScenarioArguments(route_config_list, scenario_config)
        return args

    def launchScenarioRunner(self, scenarioConfig):
        # TODO 
        scenarios = ScenarioRunner(scenarioConfig)
        scenarios.run() 

        return True

    def parseResults(self, results):
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

