---
layout: page
title: Docs
permalink: /interfaces/
main_nav: true
date:   2020-04-18T14:25:52-05:00
---

This page provides a tutorial on how to use GRAIC for developing your intelligent autonomous racing agent.

The GRAIC framework has several modules, however competitors only need to be concerned with three: (i) perception, (ii) decision & control, and (iii) vehicle inputs types. As competitors, you will be provided with the perception and vehicle API and you will build your own decision & control module.

A diagram of the module interfaces is shown below. The decision and control module built by the competitors will be just plugged into the rest of the system. Therefore, the decision & control module must take in only the perception inputs provided by the perception module and output only the inputs required by the vehicle system.

 <img src="/Race/assets/interfaces.png">

 The perception module publishes the rostopics `location`, `obstacles`, `lane_markers`, and `environment_obj_bb`. The waypoints that the competitors must reach are also provided in the rostopic `waypoint`. The decision & control module should publish the rostopics  `ackermann_control` **or** `vehicle_control`. Further details of these topics are given below.
 Again, this information shows how the GRAIC framework works, but the competitors only need to build the blue module.

Note: For all the rostopics in this document, the full rostopic name should be `/carla/<vehicle-name>/<topic-name>`. E.g. `/carla/ego_vehicle/location` will provide location information for the car named "ego_vehicle"
