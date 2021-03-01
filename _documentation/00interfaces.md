---
layout: page
title: Docs
permalink: /interfaces/
main_nav: true
---

This page provides a tutorial on how to use GRAIC for developing your intelligent autonomous racing agent.

The GRAIC framework has several modules, however competitors only need to be concerned with three: (i) perception, (ii) decision & control, and (iii) vehicle inputs types. As competitors, you will be provided with the perception and vehicle inputs types and you will build your own decision and control module.

More information can be found in the "Decision and Control" section.

<!-- The coordinate system used by ROS is ENU. The perception module is written using Python. -->

A diagram of the competition interfaces is shown below. The decision and control modules built by the competitors will merely be swapped out. Therefore, the decision and control modules must take in only the local perception provided by the perception module and output only the vehicle input required by the vehicle system.

 <img src="/Race/assets/interfaces.png">

 The perception module publishes the rostopics `location`, `obstacles`, `lane_markers`, and `environment_obj_bb`. The waypoints that the competitors must reach are also shared with the competitors in the rostopic `waypoint`. The decision and control module should publish the rostopics  `ackermann_control` **or** `vehicle_control`. Further details of these modules are detailed below.
 The rest of the framework is shown in orange. This information is shown to provide details on how the GRAIC framework works, but the competitors only need to be concerned with the modules shown in green and blue.

Note: For the rostopic here, the full rostopic name should be `/carla/<vehicle-name>/<topic-name>`. e.g. `/carla/ego_vehicle/location` will provide location information for the car named "ego_vehicle"
