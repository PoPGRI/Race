---
layout: page
title: ROS messages
main_nav: true
date:   2020-04-18T14:25:52-05:00
---

We have defined several ROS message type for the perception module to publish.

**BBInfo**
```
# Bounding box Information
string type # Type of object within bounding box
string name # Name of the object
uint32 id # Id of the object
BBSingleInfo[] vertices_locations # List of locations of the vertices
```

**BBList**
```
# Bounding box list
BBInfo[] bounding_box_vertices # List of BBInfo
```

**BBSingleInfo**
```
# Information of a single bounding box vertex
geometry_msgs/Vector3 vertex_location
```

**EvaluationInfo**
```
# Information about the evaluation of the current vehicle
int32 score # Instant score that the car has got
uint32 numObjectsHit # Number of objects the car has hit
```

**LaneInfo**
```
uint32 lane_state # Identifier of the lane that this lane marker is on; this value should be one of the numbers below

uint32 LEFT_LANE=3
unit32 CENTER_LANE=4
uint32 RIGHT_LANE=5

# Information about each lane
LaneList lane_markers_center
LaneList lane_markers_left
LaneList lane_markers_right
```

**LaneList**
```
# Location and rotation of the lane markers
geometry_msgs/Vector3[] location
geometry_msgs/Vector3[] rotation
```

**LocationInfo**
```
# Location information message type
string actor_name # Name of the actor(current car)
uint32 actor_id # ID of the actor(current car)
geometry_msgs/Vector3 location
geometry_msgs/Vector3 rotation
geometry_msgs/Vector3 velocity
```

**ObstacleInfo**
```
# Information about the obstacle
string obstacle_name # Name of the obstacle
uint32 obstacle_id # ID of the obstacle
geometry_msgs/Vector3 location
BBSingleInfo[] vertices_locations # Bounding box of the obstacle
```

**ObstacleList**
```
# List of the obstacles
ObstacleInfo[] obstacles
```

**WaypointInfo**
```
bool reachedFinal # Flag indicating if publishes final milestone waypoint
string role_name
geometry_msgs/Vector3 location # Location of the milestone waypoint
```
