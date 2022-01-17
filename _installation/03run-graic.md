---
layout: page
title: Running GRAIC
date:   2022-04-18T14:25:52-05:00
---

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
roslaunch graic_core graic_single.launch synchronous_mode_wait_for_vehicle_control_command:=True model_type:=model_free
```

A vehicle should appear in the CARLA window.

<img src="/Race/assets/baseline.png">


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
The vehicle should start moving and a score should appear when the race terminates. To run your own controller, just replace `~/workspace/graic-workspace/src/graic_core/src/baseline.py` in the above command with the path to your controller file.
