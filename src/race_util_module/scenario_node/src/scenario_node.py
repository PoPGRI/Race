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
from enum import Enum
import random

from carla_msgs.msg import CarlaCollisionEvent
from carla_msgs.msg import CarlaLaneInvasionEvent

from scenario_runner import ScenarioRunner
from srunner.scenarioconfigs.route_scenario_configuration import RouteScenarioConfiguration
from typing import List, Dict
from itertools import permutations


class ScenarioConfig:
    def __init__(self, track): 
        self.track = track
        self.scenario_config = {
            track: [
                
            ]
        }
    
    def getScenario(self, scenarios, triggerPoints):
        for config in zip(scenarios, triggerPoints):
            scenario = {
                "available_event_configurations": [
                    
                ],
                "scenario_type": config[0].value
            }
            trigger_config = {
                                "transform": {
                                    "pitch": str(config[1].rotation.pitch),
                                    "x": str(config[1].location.x),
                                    "y": str(config[1].location.y),
                                    "yaw": str(config[1].rotation.yaw),
                                    "z": str(config[1].location.z+1.22)
                                }
                            }
        
            scenario["available_event_configurations"].append(trigger_config)
            self.scenario_config[self.track].append(scenario)
        return self.scenario_config

class UnitScenarioMap(Enum):
    StationaryObjectCrossing = "ScenarioStationaryObject"
    DynamicObjectCrossing = "ScenarioDynamicObject"
    BadMerge = "ScenarioBM"
    GhostCutIn = "ScenarioGhost"
    LeadCutIn = "ScenarioLeadCut"
    LeadSlowDown = "ScenarioLeadSlow"


class ScenarioArguments:
    def __init__(self, route_config: List[RouteScenarioConfiguration], scenario_config: Dict,
        host = '127.0.0.1', port = 2000, timeout = '10.0', trafficManagerPort = '8000',
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

class ScenarioList:
    def __init__(self, track_id):
        self.unitScenarios = []
        # self.availableScenarios = []

        for scenario in UnitScenarioMap:
            self.unitScenarios.append(scenario)
            # self.availableScenarios.append(scenario)
        track_id-=1
        self.unitScenarios = self.unitScenarios[track_id:] + self.unitScenarios[:track_id]
        self.compositeScenarios = list(permutations(self.unitScenarios, 4))
    
    def getUnitScenario(self):
        scenario = self.unitScenarios.pop(0)
        # print("Get scenario: ", scenario)
        # print("Remaining scenario: ", self.unitScenarios)
        return [scenario]
    
    def hasUnitScenario(self):
        return len(self.unitScenarios) > 0

    def update(self, result):
        # NOTE scenario update heuristic:
        # 1. remove the one that fails
        # print("=========> Updating with results: ", result)
        # print("=========> available: ", self.availableScenarios)
        # if result[0] == False:
        #     if len(result[1]) == 1:
        #         self.availableScenarios.remove(result[1][0])
        pass

    def getCompositeScenario(self):
        # NOTE composite heurstics can be applied here
        if self.compositeScenarios:
            # return list(np.random.choice(self.availableScenarios, 6))
            scenario = self.compositeScenarios.pop(0)
            return scenario
        else:
            return None

class ScenarioGenerator:
    def __init__(self, track_id):
        self.scenarioList = ScenarioList(track_id)
    
    def generateScenario(self, result=None):
        if not result:
            return self.scenarioList.getUnitScenario()
        else:
            self.scenarioList.update(result)
            if self.scenarioList.hasUnitScenario():
                return self.scenarioList.getUnitScenario()
            else:
                return self.scenarioList.getCompositeScenario()

class ScenarioNode:

    def __init__(self, world, role_name='ego_vehicle', track='t1_tripe', port=2000):
        self.role_name = role_name
        self.world = world
        self.map = world.get_map()
        rospack = rospkg.RosPack()
        fpath = rospack.get_path('config_node')
        self.waypoint_list = pickle.load(open(fpath+'/'+track,'rb'))
        track_id = self.waypoint_list.pop(0)
        # self.waypoint_list = pickle.load(open(track,'rb'))
        self.map = world.get_map()
        self.ego_vehicle = None
        self.track = track
        self.wp_idx = 0
        self.collisionTest = True
        self.laneDepartureTest = True
        self.scenario = None
        self.port = port
        self.subCollision = rospy.Subscriber('/carla/%s/collision'%role_name, CarlaCollisionEvent, self.collisionCallback)
        self.subLaneInvasion = rospy.Subscriber('carla/%s/lane_invasion'%role_name, CarlaLaneInvasionEvent, self.laneCallback)
        self.scenarioGenerator = ScenarioGenerator(track_id)

        self.findEgoVehicle()

    def collisionCallback(self, data):
        self.collisionTest = False 
    
    def laneCallback(self, data):
        for marking in data.crossed_lane_markings:
            if marking is CarlaLaneInvasionEvent.LANE_MARKING_SOLID:
                self.laneDepartureTest = False

    def findEgoVehicle(self):
        for actor in self.world.get_actors():
            if actor.attributes.get('role_name') == self.role_name:
                self.ego_vehicle = actor
                break

    def createScenarioConfig(self, result=None):
        self.scenario = self.scenarioGenerator.generateScenario(result)

        if not self.scenario or self.wp_idx + 7 >= len(self.waypoint_list):
            return None
        location = carla.Location(self.waypoint_list[self.wp_idx+2][0], self.waypoint_list[self.wp_idx+2][1], self.waypoint_list[self.wp_idx+2][2])
        trigger_points = []
        for i in range(len(self.scenario)):
            if i != 0:
                trigger_point = self.map.get_waypoint(location, project_to_road=True, lane_type=carla.LaneType.Driving).next(10*i)[0].transform
            else:
                trigger_point = self.map.get_waypoint(location, project_to_road=True, lane_type=carla.LaneType.Driving).transform
            trigger_points.append(trigger_point)
        
        scenarioConfig = ScenarioConfig(self.track)
        scenario_config = scenarioConfig.getScenario(self.scenario, trigger_points)
        print("Config: ", scenario_config)
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
            z=self.waypoint_list[self.wp_idx][2]
        ))

        wp.append(carla.Location(
            x=self.waypoint_list[self.wp_idx+6][0],
            y=self.waypoint_list[self.wp_idx+6][1],
            z=self.waypoint_list[self.wp_idx+6][2]
        ))

        route_config.trajectory = wp

        route_config_list.append(route_config)

        args = ScenarioArguments(route_config_list, scenario_config, port=self.port)
        self.wp_idx += 7
        
        return args

    def launchScenarioRunner(self, scenarioConfig):
        scenarios = ScenarioRunner(scenarioConfig)
        scenarios.run() 

        return True

    def parseResults(self):
        if not self.laneDepartureTest or not self.collisionTest:
            self.laneDepartureTest = True 
            self.collisionTest = True
            return (False, self.scenario) 
        else:
            return (True, self.scenario)

def run(sn, role_name):
    result = None
    while not rospy.is_shutdown():
        scenarioConfig = sn.createScenarioConfig(result)
        if scenarioConfig:
            sn.launchScenarioRunner(scenarioConfig)
        result = sn.parseResults()

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
    sn = ScenarioNode(world, role_name, track, port)
    try:
        run(sn, role_name)
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down waypoint node")

