---
layout: page
title: Overview
---
PoPGRI uses the [CARLA simulator](https://carla.org/) alongside [ROS Noetic](https://www.ros.org/) to run synthesis algorithms. The CARLA simulator is built on the [Unreal Engine](https://www.unrealengine.com/en-US/). The instructions for CARLA installation can be found [here](https://carla.readthedocs.io/en/latest/build_linux/). The instructions for ROS Noetic installation can be found [here](http://wiki.ros.org/noetic/Installation/Ubuntu).

The software and system recommendations are listed below.

#### Software Recommendations
- Ubuntu 20.04
- Carla 0.9.11
- ROS Noetic
- Python 3

#### System Recommendations
- Intel i7 gen 9th - 11th / Intel i9 gen 9th - 11th / AMD ryzen 7 / AMD ryzen 9
- +16 GB RAM memory
- NVIDIA RTX 2070 / NVIDIA RTX 2080 / NVIDIA RTX 3070, NVIDIA RTX 3080
- 4GB GPU
- Open GL 3.3 or above​ and ​DirectX 10

## Competition Details
All submissions must be submitted as a zip file. This zip file should contain:
- Decision and control module (see Interfaces).
- Documentation including necessary libraries
- A short description of the algorithm
The competition will provide an early release of the framework to allow the competitors to run tests. This early release will contain a baseline algorithm, test vehicle, and test environment. This test vehicle and test environment are not the same as the ones provided on the day of the competition.
On the day of the competition, a final release will be provided. This final release will contain the competition vehicle and the competition environment. The competitors will have 2 hrs after the final release for submissions. These submissions will then be run by the competition hosts for scoring.

#### Scoring
The synthesis algorithms will be scored based on many factors. These factors include:
- Safety: How well can the vehicle avoid obstacles?
- Time to completion: How quickly does the vehicle complete the track?
- Computation time: How long does it take to synthesize inputs?
