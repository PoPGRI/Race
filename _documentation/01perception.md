---
layout: page
title: Perception
main_nav: true
---

The perception module consists of two rostopics: `obstacles` and `location`. These rostopics are updated at a rate of 10 Hz.

### Obstacles
The perception module returns all the obstacles within the sensing radius of the vehicle. Each obstacle has a type, id, and vertices.
- Type: The type of obstacle detected (ie. “pedestrian”, “car”, “building”, etc.)
- ID: An identifier for each obstacle (ie.  "1”, etc.). This remains the same across all time steps.
- Vertices: The location of the vertices of the bounding box of each obstacle given in [x, y, z] global coordinates

### Location
The location rostopic returns the state of the vehicle as the position, rotation, and velocity of the vehicle.
The position is given as [x, y, z] in ENU coordinates. The rotation is given as [\\( \theta_x,  \theta_y, \theta_z\\)] in radians. The velocity of the vehicle is given as [\\(v_x, v_y\\)].
