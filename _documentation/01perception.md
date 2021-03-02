---
layout: page
title: Perception
main_nav: true
---

The perception module consists of several rostopics. The information is either static or dynamic. That is, <span style="color:red">*static  information*</span> will not change over time while <span style="color:blue">*dynamic  information*</span> may change. These rostopics are updated at a rate of 20 Hz.

### Dynamic obstacles (<span style="color:blue">Dynamic</span>)
The perception module returns all the obstacles within the sensing radius of the vehicle. Each obstacle has a type, id, and vertices.
- Vertices: The location of the vertices of the bounding box of each obstacle given in \\([x, y, z]\\) global coordinates
- Type: The type of obstacle detected (ie. “pedestrian”, “car”, "motorcycle", etc.)
- ID: An identifier for each obstacle (ie.  "1”, etc.). This identifier remains the same across all time steps.

These obstacles are published to the rostopic `obstacles`.

### Static obstacles objects (<span style="color:red">Static</span>)
The environment objects include static obstacles that define the racing environment. This includes fences, sidewalks, buildings, and any other large objects. These objects are provided to the competitor as a list where each entry consists of the following:
- Vertices: The location of the vertices of the bounding box of each obstacle given in \\([x, y, z]\\) global coordinates
- Type: The type of obstacle detected (ie. "sidewalk", “fence”, “building”, etc.)
- ID: An identifier for each obstacle (ie.  "1”, etc.). This remains the same across all time steps.

These objects are also published to the rostopic `obstacles`.

### Location (<span style="color:blue">Dynamic</span>)
The location rostopic returns the state of the vehicle as the position, rotation, and velocity of the vehicle.
The position is given as [x, y, z] in ENU coordinates. The rotation is given as [\\( \theta_x,  \theta_y, \theta_z\\)] in radians. The velocity of the vehicle is given as [\\(v_x, v_y\\)].

The location is published to the rostopic `location`

### Lane markers (<span style="color:red">Static</span>)
The lane markers (or *lane markers*) describe the current lane that the vehicle is in.

The lane markers are published to the rostopic `lane_markers`.

### Milestones
These are the waypoints along the track that the vehicle must pass. These milestones are given in \\([x,y,z]\\) global coordinates and appear as a green gate on the map. The competitors must reach these milestones to score.

The milestones are published to the rostopic `waypoints`.

<img src="/Race/assets/perception_screenshot.png">

From the above image, the red bounding boxes are the obstacles; the green arcs are the milestones; the black line segments on the road are the lane markers.
There are 3 lanes in total; they are LEFT_LANE(id=3), CENTER_LANE(id=4), and RIGHT_LANE(id=5)
