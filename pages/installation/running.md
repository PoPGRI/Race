---
layout: page
title: Running GRAIC
date:   2022-04-18T14:25:52-05:00
permalink: installation/running/
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


The whole racing system consists of three independent programs, the CARLA simulator, the GRAIC infrastructure, and your controller, which should be started in three terminals. If you are using Docker, you can get a new terminal connecting to the running container by running `docker exec -it graic_con /bin/bash`, where `graic_con` is the name of the running container.

### Running the CARLA simulator
If you are using Docker, run the following command in a new terminal
```
~/workspace/carla-simulator/CarlaUE4.sh -opengl
```

If you are using AWS, run the following command in a new terminal
```
~/workspace/carla-simulator/CarlaUE4.sh
```

If everthing works, you shoule be able to see the CARLA window.

### Running the GRAIC infrastructure
First, you need to update the code. To do that, run the following command
```
~/scripts/update.sh
```

Then, run the following command
```
. ~/workspace/graic-workspace/devel/setup.bash
roslaunch graic_core graic_single.launch synchronous_mode_wait_for_vehicle_control_command:=True model_type:=model_free vis2D:=True
```

### Running the controller
The controller runs as a ROS node. The entry of the node is provided by GRAIC, which is a file called `agent_wrapper.py`. This file will import the user's controller from a file called `user_controller_file.py` and also communicate with the GRAIC infrastructure. For testing, you can use the baseline controller provided in the GRAIC repository as follows.
```
mkdir tmp
cd tmp
cp ~/workspace/graic-workspace/src/graic_core/src/agent_wrapper.py .
cp ~/workspace/graic-workspace/src/graic_core/src/baseline.py user_controller_file.py
. ~/workspace/graic-workspace/devel/setup.bash
python3 agent_wrapper.py ego_vehicle
```
The vehicle should start moving and a score should appear when the race terminates. You should be able to see the following two windows: one is for the chase camera view, and the other one is the top view 2D visualization. In the 2D visualization, the blue box is the ego vehicle, black dots are the lane markers, and red boxes are the obstacles. If you want to disable the 2D visualization, just change ```vis2D:=True``` to ```vis2D:=False``` in the above command.

<img src="/Race/assets/graic_vis.png">

To run your own controller, just replace `~/workspace/graic-workspace/src/graic_core/src/baseline.py` in the above command with the path to your controller file.

You can use the [baseline controller](https://github.com/PoPGRI/Race/blob/main/graic_core/src/baseline.py) as a template. In this file, only the `class Controller` is necessary. You can modify this class to implement your own controller. To further understand how to create controllers, take a look at the interfaces described on our [docs](https://popgri.github.io/Race/documentation/) page.
</div>
