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

The perception module consists of several rostopics. The information is either static or dynamic. That is, <span style="color:red">*static  information*</span> will not change over time while <span style="color:blue">*dynamic  information*</span> may change. These rostopics are updated at a rate of 20 Hz.

The perception distance will remained fixed across all the races. This distance is 60m.

### Dynamic obstacles (<span style="color:blue">Dynamic</span>)
The perception module returns all the obstacles within the sensing radius of the vehicle. Each obstacle has a type, id, and vertices.
- Vertices: The location of the vertices of the bounding box of each obstacle given in \\([x, y, z]\\) global coordinates
- Object center: The location of the object center in \\([x, y, z]\\) global coordinates
- Type: The type of obstacle detected (ie. “pedestrian”, “car”, "motorcycle", etc.)
- ID: An identifier for each obstacle (ie.  "1”, etc.). This identifier remains the same across all time steps.

These obstacles are published to the rostopic `obstacles`.

<!-- ### Static obstacles objects (<span style="color:red">Static</span>)
The environment objects include static obstacles that define the racing environment. This includes fences, sidewalks, buildings, and any other large objects. These objects are provided to the competitor as a list where each entry consists of the following:
- Vertices: The location of the vertices of the bounding box of each obstacle given in \\([x, y, z]\\) global coordinates
- Type: The type of obstacle detected (ie. "sidewalk", “fence”, “building”, etc.)
- ID: An identifier for each obstacle (ie.  "1”, etc.). This remains the same across all time steps.

These objects are also published to the rostopic `obstacles`. -->

### Location (<span style="color:blue">Dynamic</span>)
The location rostopic returns the state of the vehicle as the position, rotation, and velocity of the vehicle.
The position is given as [x, y, z] in ENU coordinates. The rotation is given as [\\( \theta_x,  \theta_y, \theta_z\\)] in radians. The velocity of the vehicle is given as [\\(v_x, v_y\\)].

The location is published to the rostopic `location`

### Lane markers (<span style="color:red">Static</span>)
The lane markers (or *lane markers*) describe the current lane that the vehicle is in.

The lane markers are published to the rostopic `lane_markers`.

The edges of the road are also included. The left edge is provided in `left_lane_markers` and the right edge is provided in `right_lane_markers`.

### Milestones
These are the waypoints along the track that the vehicle must pass. These milestones are given in \\([x,y,z]\\) global coordinates and appear as a green gate on the map. The competitors must reach these milestones to score.

The milestones are published to the rostopic `waypoints`.

<img src="{{site.urlimg}}perception_screenshot.png">

From the above image, the red bounding boxes are the obstacles; the green arcs are the milestones; the black line segments on the road are the lane markers.
There are 3 lanes in total; they are LEFT_LANE(id=3), CENTER_LANE(id=4), and RIGHT_LANE(id=5)

### Perception rostopics

Here is a table with the rostopics associated with perception and what they return.

|Name                |Type        |Description                                                                                   |
|:------------------:|:----------:|:---------------------------------------------------------------------------------------------|
|`obstacles`         |ObstacleList|Dynamic obstacle information                                                                  |
|`lane_markers`      |LaneInfo    |Left, center, and right lane markers that define the lane that the ego vehicle is currently on|
|`left_lane_markers` |LaneList    |Lane markers that define the outermost left edge                                              |
|`right_lane_markers`|LaneList    |Lane markers that define the outermost right edge                                             |


### 2D Visualization

We provide 2D visualization of the perception oracle as part of the simulator.
The blue box is the ego vehicle, black dots are the lane markers, and red boxes are the obstacles. If you want to disable the 2D visualization, just change ```vis2D:=True``` to ```vis2D:=False``` in the above command.

<img src="{{site.urlimg}}graic_vis.png">

</div>
