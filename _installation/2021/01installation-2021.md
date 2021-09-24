---
layout: page
title: Installation
date:   2020-04-18T14:25:52-05:00
---

### Step 0. Install the GRAIC Docker

The instructions for CARLA installation can be found [here](https://carla.readthedocs.io/en/latest/build_linux/). It is recommended to use our [GRAIC Docker](https://hub.docker.com/r/pricejiang/graic) image. The instructions here are for GRAIC installation on Ubuntu 20.04.

First, ensure that you are using the NVIDIA driver. This can be done by going to "Additional Drivers" in "Software and Updates".

 <img src="/Race/assets/driver_settings.png">

1. Install Docker CE. Instructions can be found here: [https://docs.docker.com/engine/install/ubuntu/](https://docs.docker.com/engine/install/ubuntu/)
2. Install NVIDIA-Docker2. Instructions can be found here:[https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)
3. Pull the GRAIC Docker image: `docker pull pricejiang/graic:latest`

Note that you may have to use the `sudo` command to run docker.
This may take a while.


### Step 1. Download the source code and map

Now that you have installed the GRAIC docker, you should download the source code and map for GRAIC.
These can be found as a [release](https://github.com/PoPGRI/Race/releases/tag/0.1.2) on our GitHub.
Both the source code and map are zip files, so you will need to uncompress these files.
You can extract them to any directory.

### Step 2. Run the GRAIC Docker container

Now that everything has been installed, you can now run the GRAIC docker image.
If everything works properly, once the GRAIC container is running, the baseline solution can also be run.
To run the GRAIC Docker container, use the following instructions.

Replace `[PATH-TO-THE-DOWNLOADED-CODE]` with the path to the source code and `[PATH-TO-THE-DOWNLOADED-MAP]` with the path to the downloaded map.
```
docker run --name graic_con --privileged --rm --gpus all --env NVIDIA_DISABLE_REQUIRE=1 -it --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v [PATH-TO-THE-DOWNLOADED CODE]/src:/home/carla/graic-workspace/src:rw -v [PATH-TO-THE-DOWNLOADED-MAP]/map_package/:/home/carla/carla-simulator/CarlaUE4/Content/map_package/:ro pricejiang/graic carla-simulator/CarlaUE4.sh -opengl
```
Note that you may need to use the `sudo` command to run the Docker image.

A window called CarlaUE4 will open.

<img src="/Race/assets/carlaue4.png">

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

<img src="/Race/assets/baseline.png">

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

To further understand how to create controllers, take a look at the interfaces described on our [docs](https://popgri.github.io/Race/documentation/) page.
