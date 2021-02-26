---
layout: page
title: Installation
---

### Step 0. Install the GRAIC Docker

The instructions for CARLA installation can be found [here](https://carla.readthedocs.io/en/latest/build_linux/). It is recommended to use our [GRAIC Docker](https://hub.docker.com/r/sundw2014/graic) image. The instructions here are for GRAIC installation on Ubuntu 20.04.

First, ensure that you are using the NVIDIA driver. This can be done by going to "Additional Drivers" in "Software and Updates".

1. Install Docker CE. Instructions can be found here: [https://docs.docker.com/engine/install/ubuntu/](https://docs.docker.com/engine/install/ubuntu/)
2. Install NVIDIA-Docker2. Instructions can be found here:[https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)
3. Pull the GRAIC Docker image: `docker pull sundw2014/graic:latest`

Note that you may have to use the `sudo` command to run docker.


### Step 1. Cloning the Git repo

Now that you have installed the GRAIC docker, you should clone the GRAIC repo.
Use the command `git clone https://github.com/PoPGRI/Race.git` to save the repo to any directory.

### Step 2. Installing ROS

GRAIC uses rostopics to communicate between its modules (described on the [docs](https://popgri.github.io/Race/docs/) page).
The installation instructions for ROS can be found here: [http://wiki.ros.org/noetic/Installation/Ubuntu](http://wiki.ros.org/noetic/Installation/Ubuntu).

### Step 3. Running the GRAIC Docker container

Now that everything has been installed, you can now run the GRAIC docker image.
If everything works properly, once the GRAIC container is running, the baseline solution can also be run.
To run the GRAIC Docker container, use the following instructions.

Replace `[path-to-the-cloned-graic-repo]` with the path to the cloned repository.
```
docker run --name graic_con --privileged --rm --gpus all --env NVIDIA_DISABLE_REQUIRE=1 -it --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v [path-to-the-cloned-graic-repo]/src:/home/carla/graic-workspace/src:rw sundw2014/graic:latest carla-simulator/CarlaUE4.sh
```
Note that you may need to use the `sudo` command to run the Docker image.

A window called CarlaUE4 will open.

### Step 4. Running the baseline solution

Now that the GRAIC Docker container is running, you should test that everything is working properly using the baseline solution.

You can now access the bash of the running container using `docker exec -it graic_con /bin/bash`.
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
roslaunch race carla_single.launch
```
You are now ready to start creating controllers for GRAIC!

The maps and vehicles used in GRAIC can be found on our [GitHub](https://github.com/PoPGRI/Race).

To get started with your own controller, look at the interfaces described on our [docs](https://popgri.github.io/Race/docs/) page.
