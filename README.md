# PoPGRI: A competition for intelligent racing

## Description: 

The goal of this competition is to record and help advance the state-of-the-art in (model-based and model-free) controller synthesis as applied to tactical decision making for racing environments. Comparing performance of different synthesis approaches is notoriously difficult. There are many computing platforms, approaches, no standard interfaces and problem specifications, and repeatability in research is complicated. We hope that this competition will jumpstart a solution to this problem.

The participants will be required to submit a piece of code (the synthesizer) for generating _agents_ or controllers for vehicles running in partially known, complex, simulated environments. The input to the synthesized controller will come from a perception oracle that will provide a local view of obstacles and the next sequence of gates and goals in the local coordinates of the vehicle in the current environment. The output of the synthesized controller will drive the plant (e.g., a quadcopter, or a fixed-wing aircraft). The synthesizer will take as input (a) vehicle specs: some prior knowledge or a black-box executable for simulating the plant/vehicle including the input and output interfaces and (b) track specs: parameters defining the environment, such as speed of dynamic obstacles, curvatures, etc.  With this information, the synthesizer will generate the controller which when combined with the plant and the environment will give a closed system that can be simulated. The performance evaluation will be based on simulations in a set of test environments. 

## Usage
System Requirements:
* Ubuntu 16.04
* ROS Kinetic
* Gazebo 9

To Install all dependencies, pleace refer to this [document](http://publish.illinois.edu/safe-autonomy/files/2020/08/Ubuntu-Machine-Setup.pdf)

After cloning the repository, pleace execute following command in root directory to compile and build the project
 <pre><code>catkin_make</code></pre>
 
 To launch Gazebo simulation, pleace execute following command in root directory:
 <pre><code>
 source devel/setup.bash
 roslaunch race race.launch
 </code></pre>
  
  For more information about ROS and Gazebo, please refer to their office website
  * [ROS](http://wiki.ros.org/Documentation)
  * [Gazebo](http://gazebosim.org/)
