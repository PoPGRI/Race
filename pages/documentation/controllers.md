---
layout: page
title: Decision and Control
main_nav: true
date:   2022-04-18T14:25:52-05:00
permalink: documentation/decision-control/
---
The decision and control module is built by the competitors. Any pre-computations required must be provided by the competitors, and must run without additional effort from the competition hosts. The decision and control module must output the vehicle inputs required by the vehicle system (rostopics `ackermann_control` using message type `AckermannDrive`).

Our baseline decision and controller can be found in the file `Race/src/race/src/baseline.py`.
The decision module determines what waypoint the vehicle should move to. If there is an obstacle in front of the vehicle, then the waypoint is chosen such that the vehicle can avoid it.
The controller module is a simple waypoint following controller. The competitors can use these modules as references for how to create their own controllers.
