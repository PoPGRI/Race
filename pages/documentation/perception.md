---
layout: page-fullwidth
title: Perception
main_nav: true
date:  2022-04-18T14:25:52-05:00
permalink: documentation/perception/
---

<div class="row">
<div class="medium-4 medium-push-8 columns" markdown="1">
<div class="panel radius" markdown="1">
**Table of Contents**
{: #toc }
*  TOC
{:toc}
</div>
</div><!-- /.medium-4.columns -->



<div class="medium-8 medium-pull-4 columns" markdown="1">

This year we have created a wrapper that will feed all the perception data(E.g. Obstacles, Milestones Waypoints, Velocity, Location, Lane Boundary Waypoints) into your controller. For details, you can check the code [here](https://github.com/PoPGRI/Race/blob/cfa96590c874ad96dd7671db60fc53e32dfcb286/agent.py#L8)

The perception distance for obstacles will remained fixed across all the races. This distance is 100m.

### Obstacles 
<!-- (<span style="color:blue">Dynamic</span>) -->
The perception module returns all the obstacles within the sensing radius of the vehicle. Each obstacle is a class of carla.Actor, which has a type, id, and other properties. You could find more details [here](https://carla.readthedocs.io/en/0.9.13/python_api/#carlaactor) 
<!-- - Vertices: The location of the vertices of the bounding box of each obstacle given in \\([x, y, z]\\) global coordinates
- Object center: The location of the object center in \\([x, y, z]\\) global coordinates
- Type: The type of obstacle detected (ie. “pedestrian”, “car”, "motorcycle", etc.)
- ID: An identifier for each obstacle (ie.  "1”, etc.). This identifier remains the same across all time steps. -->


### Milestones Waypoints
These are the waypoints along the track that the vehicle must pass. We provide you with all FUTURE waypoints to reach. These milestones waypoints are given in \\([x,y,z]\\) global coordinates and appear as a blue gate on the map when your vehicle approaches. The competitors must reach these milestones to score.

<!-- <img src="{{site.urlimg}}perception_screenshot.png"> -->

<!-- From the above image, the red bounding boxes are the obstacles; the green arcs are the milestones; the black line segments on the road are the lane markers. -->
<!-- There are 3 lanes in total; they are LEFT_LANE(id=3), CENTER_LANE(id=4), and RIGHT_LANE(id=5) -->


<!-- ### Static obstacles objects (<span style="color:red">Static</span>)
The environment objects include static obstacles that define the racing environment. This includes fences, sidewalks, buildings, and any other large objects. These objects are provided to the competitor as a list where each entry consists of the following:
- Vertices: The location of the vertices of the bounding box of each obstacle given in \\([x, y, z]\\) global coordinates
- Type: The type of obstacle detected (ie. "sidewalk", “fence”, “building”, etc.)
- ID: An identifier for each obstacle (ie.  "1”, etc.). This remains the same across all time steps.

These objects are also published to the rostopic `obstacles`. -->
### Velovity  
<!-- (<span style="color:blue">Dynamic</span>) -->
Velovity is directly provided at a 3D vector(Vx, Vy, Vz), you could check 
[carla.Vector3D](https://carla.readthedocs.io/en/0.9.13/python_api/#carla.Vector3D) for more details.

### Location 
<!-- (<span style="color:blue">Dynamic</span>) -->
The location contains the state of the vehicle with the position and rotation. It is a class of carla.Transform. You can read more [here](https://carla.readthedocs.io/en/0.9.13/python_api/#carla.Transform)
<!-- The location contains the state of the vehicle as the position, rotation, and velocity of the vehicle.
The position is given as [x, y, z] in ENU coordinates. The rotation is given as [\\( \theta_x,  \theta_y, \theta_z\\)] in radians. The velocity of the vehicle is given as [\\(v_x, v_y\\)]. -->

### Lane Boundary Waypoints
<!-- (<span style="color:red">Static</span>) -->
This tells you information aboout the current lane your vehicle is in. This contains 20 waypoinst on the left and right lane boundary for the next 20 meters.

<!-- The edges of the road are also included. The left edge is provided in `left_lane_markers` and the right edge is provided in `right_lane_markers`. -->



<!-- 
### 2D Visualization

We provide 2D visualization of the perception oracle as part of the simulator.
The blue box is the ego vehicle, black dots are the lane markers, and red boxes are the obstacles. If you want to disable the 2D visualization, just change ```vis2D:=True``` to ```vis2D:=False``` in the above command.

<img src="{{site.urlimg}}graic_vis.png"> -->

</div>
