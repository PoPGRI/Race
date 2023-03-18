import numpy as np
import time
import pickle
import carla
import os
from enum import Enum
import random

from scenario_runner import ScenarioRunner
from srunner.scenarioconfigs.route_scenario_configuration import RouteScenarioConfiguration
from typing import List, Dict
from itertools import permutations

import argparse


def get_open_port():
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("", 0))
    s.listen(1)
    port = s.getsockname()[1]
    s.close()
    return port


class ScenarioConfig:
    def __init__(self, track):
        self.track = track
        self.scenario_config = {track: []}

    def getScenario(self, scenarios, triggerPoints):
        for config in zip(scenarios, triggerPoints):
            scenario = {
                "available_event_configurations": [],
                "scenario_type": config[0].value
            }
            trigger_config = {
                "transform": {
                    "pitch": str(config[1].rotation.pitch),
                    "x": str(config[1].location.x),
                    "y": str(config[1].location.y),
                    "yaw": str(config[1].rotation.yaw),
                    "z": str(config[1].location.z + 1.22)
                }
            }

            scenario["available_event_configurations"].append(trigger_config)
            self.scenario_config[self.track].append(scenario)
        return self.scenario_config


class UnitScenarioMap(Enum):
    # StationaryObjectCrossing = "ScenarioStationaryObject"
    DynamicObjectCrossing = "ScenarioDynamicObject"
    BadMerge = "ScenarioBM"
    GhostCutIn = "ScenarioGhost"
    LeadCutIn = "ScenarioLeadCut"
    LeadSlowDown = "ScenarioLeadSlow"


class ScenarioArguments:
    def __init__(self,
                 route_config: List[RouteScenarioConfiguration],
                 scenario_config: Dict,
                 host='127.0.0.1',
                 port=2000,
                 timeout='1000.0',
                 trafficManagerPort='8000',
                 trafficManagerSeed='0',
                 sync=False,
                 agent=None,
                 agentConfig='',
                 output=True,
                 result_file=False,
                 junit=False,
                 json=False,
                 outputDir='',
                 configFile='',
                 additionalScenario='',
                 debug=False,
                 reloadWorld=False,
                 record='',
                 randomize=False,
                 repetitions=1,
                 waitForEgo=False):
        self.host = host
        self.port = port
        self.timeout = timeout
        # self.trafficManagerPort = trafficManagerPort
        self.trafficManagerPort = get_open_port()
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
        track_id -= 1
        self.unitScenarios = self.unitScenarios[
            track_id:] + self.unitScenarios[:track_id]
        self.compositeScenarios = list(permutations(self.unitScenarios, 2))

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
    def __init__(self,
                 world,
                 role_name='ego_vehicle',
                 track='t1_tripe',
                 port=2000):
        self.role_name = role_name
        self.world = world
        self.map = world.get_map()

        self.waypoint_list = pickle.load(open("./waypoints/{}".format(track), 'rb'))
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
        self.scenarioGenerator = ScenarioGenerator(track_id)

        self.findEgoVehicle()
        print(self.ego_vehicle)

    def findEgoVehicle(self):
        for actor in self.world.get_actors():
            if actor.attributes.get('role_name') == self.role_name:
                self.ego_vehicle = actor
                break

    def createScenarioConfig(self, result=None):
        self.scenario = self.scenarioGenerator.generateScenario(result)

        if not self.scenario or self.wp_idx + 7 >= len(self.waypoint_list):
            return None
        location = carla.Location(self.waypoint_list[self.wp_idx + 2][0],
                                  self.waypoint_list[self.wp_idx + 2][1],
                                  self.waypoint_list[self.wp_idx + 2][2])
        trigger_points = []
        for i in range(len(self.scenario)):
            if i != 0:
                trigger_point = self.map.get_waypoint(
                    location,
                    project_to_road=True,
                    lane_type=carla.LaneType.Driving).next(10 * i)[0].transform
            else:
                trigger_point = self.map.get_waypoint(
                    location,
                    project_to_road=True,
                    lane_type=carla.LaneType.Driving).transform
            trigger_points.append(trigger_point)

        scenarioConfig = ScenarioConfig(self.track)
        scenario_config = scenarioConfig.getScenario(self.scenario,
                                                     trigger_points)
        print("Config: ", scenario_config)
        route_config_list = []
        route_config = RouteScenarioConfiguration()
        route_config.town = self.track
        route_config.name = 'RouteScenario_0'
        route_config.weather = carla.WeatherParameters(sun_altitude_angle=70)
        # route_config.weather = carla.WeatherParameters(sun_altitude_angle=70)
        route_config.scenario_config = scenario_config
        wp = []
        wp.append(
            carla.Location(x=self.waypoint_list[self.wp_idx][0],
                           y=self.waypoint_list[self.wp_idx][1],
                           z=self.waypoint_list[self.wp_idx][2]))

        wp.append(
            carla.Location(x=self.waypoint_list[self.wp_idx + 6][0],
                           y=self.waypoint_list[self.wp_idx + 6][1],
                           z=self.waypoint_list[self.wp_idx + 6][2]))

        route_config.trajectory = wp

        route_config_list.append(route_config)

        args = ScenarioArguments(route_config_list,
                                 scenario_config,
                                 port=self.port)
        self.wp_idx += 7

        return args

    def launchScenarioRunner(self, scenarioConfig):
        scenarios = ScenarioRunner(scenarioConfig)
        print(scenarioConfig)
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
    while True:
        scenarioConfig = sn.createScenarioConfig(result)
        if scenarioConfig:
            sn.launchScenarioRunner(scenarioConfig)
            result = sn.parseResults()
        else:
            break


if __name__ == "__main__":
    argparser = argparse.ArgumentParser(
        description='CARLA Automatic Control Client')
    argparser.add_argument(
        '-m', '--map',
        help='Set Different Map for testing: shanghai_intl_circuit, t1_triple, t2_triple, t3, t4',
        default="shanghai_intl_circuit")
    args = argparser.parse_args()

    host = '127.0.0.1'
    port = 2000
    role_name = "ego_vehicle"
    track = args.map
    client = carla.Client(host, port)
    world = client.get_world()
    sn = ScenarioNode(world, role_name, track, port)
    # try:
    run(sn, role_name)
    # except Exception as e:
    #     print(e)