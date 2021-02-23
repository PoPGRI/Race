---
layout: page
title: Installation
---

These instructions are adapted from the [Carla Documentation](https://carla.readthedocs.io/en/latest/build_docker/) and [this article](https://antc2lt.medium.com/carla-on-ubuntu-20-04-with-docker-5c2ccdfe2f71).

### Installing the Carla Docker

First, ensure that you are using the NVIDIA driver. This can be done by going to "Additional Drivers" in "Software and Updates".

1. Install Docker CE. Instructions can be found here: [https://docs.docker.com/engine/install/ubuntu/](https://docs.docker.com/engine/install/ubuntu/)
2. Install NVIDIA-Docker2. Instructions can be found here:[https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)
3. Pull the Carla image: `docker pull carlasim/carla:version`

You can now run the Docker container.
```
docker run \
 -p 2000-2002:2000-2002 \
 --runtime=nvidia \
 --gpus all \
 -e DISPLAY=$DISPLAY \
 -v /tmp/.X11-unix:/tmp/.X11-unix \
 -it \
 carlasim/carla \
 bash
```

### Using the PythonAPI

You can export the PythonAPI folder using the following command.
```
docker cp <container id>:<src path> <dst path>
```

Here, the container id can be found when you enter Carla bash (ie. `docker cp <container id>:<src path> <dst path>`) or by running the command `docker ps -a` (if you have already closed the docker).

The source path is `/home/carla/PythonAPI`.

You will have to use Python 3.7. To run a python script in Carla, you can start the Carla container in one terminal, then run the python script. To run the Carla container, use the following commands.
```
docker run \
 -p 2000-2002:2000-2002 \
 --cpuset-cpus="0-5" \
 --runtime=nvidia \
 --gpus 'all,"capabilities=graphics,utility,display,video,compute"' \
 -e DISPLAY=$DISPLAY \
 -v /tmp/.X11-unix:/tmp/.X11-unix \
 -it \
 carlasim/carla \
 ./CarlaUE4.sh -opengl $1
```

### Installing ROS

The installation instructions for ROS can be found here: [http://wiki.ros.org/noetic/Installation/Ubuntu](http://wiki.ros.org/noetic/Installation/Ubuntu).
