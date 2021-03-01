---
layout: page
title: ROS Messages
main_nav: true
---

We have defined several ROS message type for the perception module to publish. 

**BBInfo**
<pre><code># Bounding box Information
string type # Type of object within bounding box
string name # Name of the object
uint32 id # Id of the object
BBSingleInfo[] vertices_locations # List of locations of the vertices </code></pre>

**BBList**
<pre><code># Bounding box list
BBInfo[] bounding_box_vertices # List of BBInfo </code></pre>

**BBSingleInfo**
<pre><code># Information of a single bounding box vertex
geometry_msgs/Vector3 vertex_location
</code></pre>

**EvaluationInfo**
<pre><code># Information about the evaluation of the current vehicle
int32 score # Instant score that the car has got
uint32 numObjectsHit # Number of objects the car has hit
</code></pre>

**LaneInfo**
<pre><code># Information about a lane marker 
geometry_msgs/Vector3 location # Location of the lane marker
geometry_msgs/Vector3 rotation # Rotation of the lane marker

uint32 lane_state # Identifier of the lane that this lane marker is on; this value should be one of the numbers below

uint32 LEFT_LANE=3 
unit32 CENTER_LANE=4
uint32 RIGHT_LANE=5
</code></pre>

**LaneList**
<pre><code># Lane marker list
LaneInfo[] lane_markers # A list of lane_markers
</code></pre>

**LocationInfo**
<pre><code># Location information message type
string actor_name # Name of the actor(current car)
uint32 actor_id # ID of the actor(current car)
geometry_msgs/Vector3 location
geometry_msgs/Vector3 rotation
geometry_msgs/Vector3 velocity
</code></pre>

**ObstacleInfo**
<pre><code># Information about the obstacle
string obstacle_name # Name of the obstacle
uint32 obstacle_id # ID of the obstacle
geometry_msgs/Vector3 location
BBSingleInfo[] vertices_locations # Bounding box of the obstacle
</code></pre>

**ObstacleList**
<pre><code> # List of the obstacles
ObstacleInfo[] obstacles
</code></pre>
