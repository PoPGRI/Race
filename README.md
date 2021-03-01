# GRAIC: A competition for intelligent racing

## Description: 

Generalized RAcing Intelligence Competition (GRAIC). The goal of this competition is to record and help advance the state-of-the-art in (model-based and model-free)  as decision & control applied to tactical decision making for racing environments. Comparing performance of different decision & control approaches is notoriously difficult. There are many computing platforms, approaches, no standard interfaces and problem specifications, and repeatability in research is complicated. We hope that this competition will jumpstart a solution to this problem.

The participants will be required to submit a piece of code (decision & control module) for generating controllers for vehicles running in partially known, complex, simulated environments. The input to the synthesized controller will come from a perception oracle that will provide a local view of obstacles and the next sequence of gates and goals in the local coordinates of the vehicle in the current environment. The output of the controller will drive the plant (e.g., a quadcopter, or a fixed-wing aircraft). The decision & control module will take as input (a) vehicle specs: some prior knowledge or a black-box executable for simulating the plant/vehicle including the input and output interfaces and (b) track specs: parameters defining the environment, such as speed of dynamic obstacles, curvatures, etc.  With this information, the decision & control module will generate the inputs which when combined with the plant and the environment will give a closed system that can be simulated. The performance evaluation will be based on simulations in a set of test environments. 

### System recommendation

GRAIC 2021 uses the [CARLA simulator](https://carla.org/), [ROS Noetic](https://www.ros.org/) and CARLA is built on the [Unreal Engine](https://www.unrealengine.com/en-US/). That is why you will want to run the simulator on a machine with a at least 16GB RAM and a powerful GPU.

#### Hardware
- Intel i7 gen / i9 gen / AMD ryzen 7 / ryzen 9, with at least 16 GB RAM
- Nvidia RTX 2070 / 2080 / 3070 / 3080 with 4GB GPU video RAM

#### Software
- Ubuntu 20.04
- Carla 0.9.11
- ROS Noetic
- Python 3.7
- Open GL 3.3 or above and DirectX 10

We will not be able to provide much support for running GRAIC on other software configurations. 

### Step 0. Install the GRAIC Docker

The instructions for CARLA installation can be found [here](https://carla.readthedocs.io/en/latest/build_linux/). It is recommended to use our [GRAIC Docker](https://hub.docker.com/r/sundw2014/graic) image. The instructions here are for GRAIC installation on Ubuntu 20.04.

First, ensure that you are using the NVIDIA driver. This can be done by going to "Additional Drivers" in "Software and Updates".

![DriverSettings](https://raw.githubusercontent.com/PoPGRI/Race/gh-pages/assets/driver_settings.png)

1. Install Docker CE. Instructions can be found here: [https://docs.docker.com/engine/install/ubuntu/](https://docs.docker.com/engine/install/ubuntu/)
2. Install NVIDIA-Docker2. Instructions can be found here:[https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)
3. Pull the GRAIC Docker image: `docker pull sundw2014/graic:latest`

Note that you may have to use the `sudo` command to run docker.
This may take a while.


### Step 1. Download the source code and map

Now that you have installed the GRAIC docker, you should download the source code and map for GRAIC.
These can be found as a [release](https://github.com/PoPGRI/Race/releases/tag/0.1.0) on our GitHub.
Both the source code and map are zip files, so you will need to uncompress these files.
You can extract them to any directory.

### Step 2. Run the GRAIC Docker container

Now that everything has been installed, you can now run the GRAIC docker image.
If everything works properly, once the GRAIC container is running, the baseline solution can also be run.
To run the GRAIC Docker container, use the following instructions.

Replace `[PATH-TO-THE-DOWNLOADED-CODE]` with the path to the source code and `[PATH-TO-THE-DOWNLOADED-MAP]` with the path to the downloaded map.
```
docker run --name graic_con --privileged --rm --gpus all --env NVIDIA_DISABLE_REQUIRE=1 -it --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v [PATH-TO-THE-DOWNLOADED CODE]/src:/home/carla/graic-workspace/src:rw -v [PATH-TO-THE-DOWNLOADED-MAP]/map_package/:/home/carla/carla-simulator/CarlaUE4/Content/map_package/:ro sundw2014/graic carla-simulator/CarlaUE4.sh -opengl
```
Note that you may need to use the `sudo` command to run the Docker image.

A window called CarlaUE4 will open.

![DriverSettings](https://raw.githubusercontent.com/PoPGRI/Race/gh-pages/assets/carlaue4.png)

### Step 3. Run the baseline solution

Now that the GRAIC Docker container is running, you should test that everything is working properly using the baseline solution.

You can now access the bash of the running container using `docker exec -it graic_con /bin/bash` in a new terminal.
If you have connected to the bash properly, then you will see the id of the container followed by `$:`.

You can run our baseline solution using the following commands.
```
cd graic-workspace
```
```
catkin_make
```
```
. devel/setup.bash
```
```
roslaunch race carla_single.launch num_wheels:=4 model_type:=model_free
```

A vehicle should appear in the CARLA window.

![DriverSettings](https://raw.githubusercontent.com/PoPGRI/Race/gh-pages/assets/baseline.png)

Now open a new and use `docker exec -it graic_con /bin/bash` and run the following in the CARLA container.
```
cd graic-workspace/
```
```
. devel/setup.bash
```
```
cd src/race/src/
```
```
python baseline.py
```
The vehicle should start moving and a score should appear.

You are now ready to start creating controllers for GRAIC!

### Step 4. Create your own controller

We have provided a template controller for you to get started on creating your own controller.
This template can be found on our [GitHub](https://github.com/PoPGRI/Race) and it is called `src/race/src/starter.py`.
You will not have to edit any part of the file other than line 66. You should replace `TODO Add your decision logic here` with the decision logic of your controller.

To further understand how to create controllers, take a look at the interfaces described on our [docs](https://popgri.github.io/Race/docs/) page.

