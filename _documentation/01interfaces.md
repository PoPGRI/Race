---
layout: page
title: Interfaces
permalink: /interfaces/
main_nav: true
---
PoPGRI reduces an autonomous system into 3 submodules: (i) perception, (ii) decision and control, and (iii) vehicle system. The competitors are provided with the perception and vehicle systems. The competitors must build their own decision and control modules.
The coordinate system used by ROS is ENU. The perception module is written using Python.
A diagram of the competition interfaces is shown below. The decision and control modules built by the competitors will merely be swapped out. Therefore, the decision and control modules must take in only the local perception provided by the perception module and output only the vehicle input required by the vehicle system.

 <img src="/Race/assets/interface.png">

### Perception
The perception used in PoPGRI is split into two components. The first components of perception returns the obstacles that are within the sensing radius of the vehicle. The second component of perception returns the current task that the vehicle is trying to achieve along with a "lane"-view of the track it is on.

#### Obstacles
The perception module returns all the obstacles within the sensing radius of the vehicle. Each obstacle has a type, id, and location. The perception can be retrieved by subscribing to the rostopic `obstacles`.
- Type: The type of obstacle detected (ie. “pedestrian”, “car”, “building”, etc.)
- ID: An identifier for each obstacle (ie. “pedestrian 1”, etc.). This remains the same across all time steps.
- Location: The location of the center of each obstacle given in [x, y, z] global coordinates

*The bounding box information will be updated soon.*


#### Lanes
The current lane that the vehicle is in is described by a list of points between the current location of the vehicle and the end of the lane. This information is published to the rostopic `lane-waypoints`.

### Decision and Control
The decision and control module is built by the competitors. All pre-computations required for the synthesis algorithms must be provided by the competitors, and must run without additional effort from the competition hosts. The decision and control module must output the vehicle inputs required by the vehicle system. For more information, please refer to the diagram.
The decision and control module should output the rostopic `vehicle_input`.

### Vehicle System
The vehicle system provided by CARLA. The vehicle system is subscribed to the rostopic `vehicle_input`.
