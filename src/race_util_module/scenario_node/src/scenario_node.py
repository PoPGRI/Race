import rospy 
import numpy as np
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
from graic_msgs.msg import WaypointInfo
import time
import pickle
import carla
import os
from enum import Enum

from carla_msgs.msg import CarlaCollisionEvent
from carla_msgs.msg import CarlaLaneInvasionEvent

from scenario_runner import ScenarioRunner
from srunner.scenarioconfigs.route_scenario_configuration import RouteScenarioConfiguration
from typing import List, Dict



class ScenarioConfig:
    def __init__(self, track, scenario, triggerPoint): 
        self.scenario_config = {
            track: [
                {
                    "available_event_configurations": [
                        {
                            "transform": {
                                "pitch": "0",
                                "x": str(triggerPoint.location.x),
                                "y": str(triggerPoint.location.y),
                                "yaw": str(triggerPoint.rotation.yaw),
                                "z": "1.22"
                            }
                        }
                    ],
                    "scenario_type": scenario.value
                }
            ]
        }

class UnitScenarioMap(Enum):
    BadMerge = "ScenarioBM"
    GhostCutIn = "GhostCutIn"
    ScenarioA = "ScenarioA"
    ScenarioB = "ScenarioB"

class ScenarioCategory(Enum):
    UnitScenario = 0
    CompositeScenario = 1

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

# class ScenarioType(Enum):
#     UnitScenario = 0
#     CompositeScenario = 1

# class Scenario:
#     def __init__(self, location, category, names, startPoint, endPoint):
#         self.location = location 
#         self.category = category
#         self.names = names
#         self.startPoint = startPoint
#         self.endPoint = endPoint

#         if self.category == ScenarioCategory.UnitScenario:
#             assert(len(self.names) == 1)

#         for name in self.names:
#             if not name in UnitScenarioMap:
#                 raise RuntimeError("Unknown Unit Scenario, please check the input")


class ScenarioList:
    def __init__(self):
        self.unitScenarios = ["ScenarioBM", "ScenarioBM"]
        self.compositeScenarios = []
        self.availableScenarios = ["ScenarioBM"]

        # for name in UnitScenarioMap:
        #     scenario = Scenario()
        #     self.unitScenarios.append(scenario)
        #     self.availableScenarios.append(scenario)
    
    def getUnitScenario(self):
        scenario = self.unitScenarios.pop(0)
        return scenario
    
    def hasUnitScenario(self):
        return len(self.unitScenarios) > 0

    def removeScenarios(self, result):
        if result[0] == False:
            self.availableScenarios.remove(result[1])

    def getCompositeScenario(self):
        # NOTE some heurstics can be applied here
        return scenario

class ScenarioGenerator:
    def __init__(self, config):
        self.scenarioList = ScenarioList()
    
    def generateScenario(self, ego_vehicle_status, result=None):
        if not result:
            return self.scenarioList.getUnitScenario()
        else:
            self.scenarioList.removeScenarios(result)
            if self.scenarioList.hasUnitScenario():
                return self.scenarioList.getUnitScenario()
            else:
                return self.scenarioList.getCompositeScenario()

class ScenarioNode:

    def __init__(self, world, role_name='ego_vehicle', track='t1_tripe'):
        self.role_name = role_name
        self.world = world
        self.map = world.get_map()
        self.waypoint_list = pickle.load(open(track,'rb'))
        self.map = world.get_map()
        self.ego_vehicle = None
        self.track = track
        self.wp_idx = 0
        self.collisionTest = True
        self.laneDepartureTest = True
        self.subCollision = rospy.Subscriber('/carla/%s/collision'%role_name, CarlaCollisionEvent, self.collisionCallback)
        self.subLaneInvasion = rospy.Subscriber('carla/%s/lane_invasion'%role_name, CarlaLaneInvasionEvent, self.laneCallback)
        # self.subWaypoint = rospy.Subscriber('/carla/%s/waypoints'%role_name, WaypointInfo, self.waypointCallback)
        self.scenarioGenerator = ScenarioGenerator(None)

        self.findEgoVehicle()

    def collisionCallback(self, data):
        self.collisionTest = False 
    
    def laneCallback(self, data):
        self.laneDepartureTest = False

    # def waypointCallback(self, data):


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
        # self.scenarioGenerator.generateScenario(self.getVehicleStatus(), result)
        # if result is None:
        if self.wp_idx + 7 >= len(self.waypoint_list):
            return None
        location = carla.Location(self.waypoint_list[self.wp_idx+4][0], self.waypoint_list[self.wp_idx+4][1], self.waypoint_list[self.wp_idx+4][2])
        trigger_point = self.map.get_waypoint(location, project_to_road=True, lane_type=carla.LaneType.Driving).transform
        # trigger_point = self.waypoint_list[4]
        scenarioConfig = ScenarioConfig(self.track, UnitScenarioMap.BadMerge, trigger_point)
        scenarioConfig.scenario_config
        scenario_config = scenarioConfig.scenario_config

        route_config_list = []
        route_config = RouteScenarioConfiguration()
        route_config.town = self.track
        route_config.name = 'RouteScenario_0'
        route_config.weather = carla.WeatherParameters(sun_altitude_angle=70)
        route_config.scenario_config = scenario_config
        wp = []
        wp.append(carla.Location(
            x=self.waypoint_list[self.wp_idx][0],
            y=self.waypoint_list[self.wp_idx][1],
            z=0.0
        ))

        wp.append(carla.Location(
            x=self.waypoint_list[self.wp_idx+6][0],
            y=self.waypoint_list[self.wp_idx+6][1],
            z=0.0
        ))

        route_config.trajectory = wp

        route_config_list.append(route_config)

        args = ScenarioArguments(route_config_list, scenario_config)
        self.wp_idx += 7
        
        return args

    def launchScenarioRunner(self, scenarioConfig):
        # TODO 
        scenarios = ScenarioRunner(scenarioConfig)
        scenarios.run() 

        return True

    def parseResults(self):
        if not self.laneDepartureTest or not self.collisionTest:
            # Failed scenario
            # Perform composite clean
            return False 
        else:
            return True

def run(sn, role_name):
    # rate = rospy.Rate(1)  # 20 Hz    
    # pubWaypoint = rospy.Publisher('/carla/%s/waypoints'%role_name, WaypointInfo, queue_size=None)
    # Send out all unit scenariose
    # scenarioConfig = sn.createScenarioConfig()
    # firstResults = sn.launchScenarioRunner(scenarioConfig)
    # sn.parseResults()
    # scenarioConfig = sn.createScenarioConfig(firstResults)
    # secondResults = sn.launchScenarioRunner(scenarioConfig)
    # sn.parseResults()

    # NOTE Question: Do we need to send out scores often? 
    while not rospy.is_shutdown():
        scenarioConfig = sn.createScenarioConfig()
        if scenarioConfig:
            sn.launchScenarioRunner(scenarioConfig)
        # rate.sleep()

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

