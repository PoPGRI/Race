---
layout: page
title: Docs
main_nav: true
date:   2022-04-18T14:25:52-05:00
---

This page provides a tutorial on how to use GRAIC for developing your intelligent autonomous racing agent.

The GRAIC framework has several modules, however competitors only need to be concerned with three: (i) perception, (ii) decision & control, and (iii) vehicle inputs types. As competitors, you will be provided with the perception and vehicle API and you will build your own decision & control module.

A diagram of the module interfaces is shown below. The decision and control module built by the competitors will be just plugged into the rest of the system. Therefore, the decision & control module must take in only the perception inputs provided by the perception module and output only the inputs required by the vehicle system.

 <img src="/Race/assets/interfaces.png">

Specifically, your controller should implement a class called `Controller` as follows. You can find an example in this [file](https://github.com/PoPGRI/Race/blob/main/graic_core/src/baseline.py).
```Python
class Controller(object):
    """docstring for Controller"""
    def __init__(self):
        super(Controller, self).__init__()

    def execute(self, currState, obstacleList, lane_marker, waypoint):
        return ...
```

As shown above, the ```exectute``` function will be called at every time step. It takes in the current state of the vehicle, a list of obstacles, a list of lane markers, and a waypoint. It should return either ```None``` or an ```AckermannDrive``` object. If it returns ```None```, the race will be immediately terminated.
