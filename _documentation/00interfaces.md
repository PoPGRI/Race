---
layout: page
title: Docs
permalink: /interfaces/
main_nav: true
---

This page provides a tutorial on how to use GRAIC for developing your intelligent autonomous racing agent. 

GRAIC reduces an autonomous system into 3 submodules: (i) perception, (ii) decision & control, and (iii) vehicle and environment system. As competitors, you will be provided with the perception and vehicle systems and you will build your own decision and control module. 

More information can be found in the "Decision and Control" section.

<!-- The coordinate system used by ROS is ENU. The perception module is written using Python. -->

A diagram of the competition interfaces is shown below. The decision and control modules built by the competitors will merely be swapped out. Therefore, the decision and control modules must take in only the local perception provided by the perception module and output only the vehicle input required by the vehicle system.

 <img src="/Race/assets/interfaces.png">

 The perception module publishes the rostopics `location`, `obstacles`, `environment_obj_bb`. The waypoints that the competitors must reach are also shared with the competitors in a file called `waypoint_list.py`. The decision and control module should publish the rostopics  `ackermann_cmd` **or** `vehicle_control_cmd`. Further details of these modules are detailed below.
