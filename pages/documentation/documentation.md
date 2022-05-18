---
layout: page
show_meta: false
title: "Documentation"
header:
   image_fullwidth: "new-header.gif"
permalink: "/documentation/"
---

In order to make benchmarking feasible, GRAIC has defined three main interfaces of an autonomous system:
- Perception
- Decision and control
- Vehicle system
These interfaces can be seen in the image below.
In GRAIC, the ground-truth perception information is provided to the competitors.
The competitors will then build a decision and control method which will compute a control input the the vehicle system.
This control input is then given to the vehicle system, and the state of the vehicle is then updated.
<img src="{{site.urlimg}}interface.png">

This page provides a general explanation on how to use GRAIC for developing your intelligent autonomous racing agent.
Please see our other documentation pages for more detailed information.

The GRAIC framework has several modules, however competitors only need to be concerned with three: (i) perception, (ii) decision & control, and (iii) vehicle inputs types. As competitors, you will be provided with the perception and vehicle API and you will build your own decision & control module.

A diagram of the module interfaces is shown below. The decision and control module built by the competitors will be just plugged into the rest of the system. Therefore, the decision & control module must take in only the perception inputs provided by the perception module and output only the inputs required by the vehicle system.

 <img src="{{site.urlimg}}interfaces.png">

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

In detail, ```currState``` is a tuple which has three elements: (position, rotation, velocity), and each element is a tuple consisting of the coordinates of that element. So position is a tuple of the form (x, y), which denotes the x and y coordinates of the vehicle; rotation is a tuple of the form (x, y, z), in radians, which denotes the roll, pitch and yaw of the vehicle; velocity is a tuple of the form (x, y), which denotes the x and y components of the velocity of the vehicle. The function ```locationCallback``` inside the class ```VehiclePerception``` in the file ```agent_wrapper.py``` provides a more starightforward way to understand the structure of ```currState```.

```obstacleList``` is a list of msg of the type ```ObstacleInfo``` (all msg structures can be found in graic_msg/msg.) The ith obstacle can be accessed by  ```obstacleList[i]``` and it has four fields: obstacle_name (type: string), obstacle_id (type: unit32), location (type: geometry_msgs/Vector3), vertices_locations (type: BBSingleInfo[]). For example, ```obstacleList[i].obstacle_name``` will return the name of the obstacle.

```lane_marker``` consists of three ```LaneList```s: lane_markers_center, lane_markers_left, and lane_markers_right. For example, ```lane_marker.lane_markers_left``` returns lane_markers_left. Note that,  ```lane_marker``` only contains markers of the lane on which the vehicle is currently driving. To access the lane markers of the boundaries of the track, please check rostopics named left_lane_markers and right_lane_markers.

```waypoint``` contains the next waypoint that the vehicle is expected to pass. For example, ```waypoint.location.x``` returns the x coordinate of the waypoint, and similarly for the y coordinate.

```exectute``` is expected to return either ```None``` or an ```AckermannDrive``` object. If it returns ```None```, exit(0) will be called and the race will terminate, otherwise the returned object will be published to control the vehicle (see publish_control function in agent_wrapper.py).
