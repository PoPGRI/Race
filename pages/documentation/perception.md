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

This year we have created a wrapper that will feed all the perception data(E.g. Obstacles, Location, Lane Markers and Milestones) into your controller. For details, you can check the code [here](https://github.com/PoPGRI/Race/blob/7fe1f4764436187b5f5b785d9a8bc2741ebe86c7/graic_core/src/agent_wrapper.py#L110)

The perception distance will remained fixed across all the races. This distance is 60m.

### Dynamic obstacles (<span style="color:blue">Dynamic</span>)
The perception module returns all the obstacles within the sensing radius of the vehicle. Each obstacle has a type, id, and vertices.
- Vertices: The location of the vertices of the bounding box of each obstacle given in \\([x, y, z]\\) global coordinates
- Object center: The location of the object center in \\([x, y, z]\\) global coordinates
- Type: The type of obstacle detected (ie. “pedestrian”, “car”, "motorcycle", etc.)
- ID: An identifier for each obstacle (ie.  "1”, etc.). This identifier remains the same across all time steps.


<!-- ### Static obstacles objects (<span style="color:red">Static</span>)
The environment objects include static obstacles that define the racing environment. This includes fences, sidewalks, buildings, and any other large objects. These objects are provided to the competitor as a list where each entry consists of the following:
- Vertices: The location of the vertices of the bounding box of each obstacle given in \\([x, y, z]\\) global coordinates
- Type: The type of obstacle detected (ie. "sidewalk", “fence”, “building”, etc.)
- ID: An identifier for each obstacle (ie.  "1”, etc.). This remains the same across all time steps.

These objects are also published to the rostopic `obstacles`. -->

### Location (<span style="color:blue">Dynamic</span>)
The location contains the state of the vehicle as the position, rotation, and velocity of the vehicle.
The position is given as [x, y, z] in ENU coordinates. The rotation is given as [\\( \theta_x,  \theta_y, \theta_z\\)] in radians. The velocity of the vehicle is given as [\\(v_x, v_y\\)].

### Lane markers (<span style="color:red">Static</span>)
The lane markers (or *lane markers*) describe the current lane that the vehicle is in.

The edges of the road are also included. The left edge is provided in `left_lane_markers` and the right edge is provided in `right_lane_markers`.

### Milestones
These are the waypoints along the track that the vehicle must pass. These milestones are given in \\([x,y,z]\\) global coordinates and appear as a green gate on the map. The competitors must reach these milestones to score.

<img src="{{site.urlimg}}perception_screenshot.png">

From the above image, the red bounding boxes are the obstacles; the green arcs are the milestones; the black line segments on the road are the lane markers.
There are 3 lanes in total; they are LEFT_LANE(id=3), CENTER_LANE(id=4), and RIGHT_LANE(id=5)


### 2D Visualization

We provide 2D visualization of the perception oracle as part of the simulator.
The blue box is the ego vehicle, black dots are the lane markers, and red boxes are the obstacles. If you want to disable the 2D visualization, just change ```vis2D:=True``` to ```vis2D:=False``` in the above command.

<img src="{{site.urlimg}}graic_vis.png">

</div>
