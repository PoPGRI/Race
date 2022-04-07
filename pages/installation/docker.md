---
layout: page-fullwidth
title: Installing GRAIC's Docker image on a local machine
date:   2022-04-18T14:25:52-05:00
permalink: installation/docker/
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

This section provide another option such that you can run GRAIC on your local machine with Docker.

GRAIC 2022 uses the [CARLA simulator](https://carla.org/), and CARLA is built on the [Unreal Engine](https://www.unrealengine.com/en-US/). That is why you will want to run the simulator on a machine with a powerful GPU. As for the software, you need the [nvidia-docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker).

### Step 1. Installing nvidia-docker and getting the GRAIC docker image
Please follow the official tutorial to install the [nvidia-docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker).

After the installation, pull the GRAIC Docker image:
{% include alert terminal='docker pull sundw2014/graic:latest' %}

### Step 2. Starting a Docker container

Now that everything has been installed, you can now run the GRAIC docker image. To run the GRAIC Docker container, use the following instructions.

{% include alert terminal='sudo xhost +' %}
{% include alert terminal='docker run --name graic_con --privileged --rm --gpus all --env NVIDIA_DISABLE_REQUIRE=1 -it --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw sundw2014/graic /bin/bash
' %}

Now the container is running, you can follow the [instructions](running.md) to run GRAIC inside the container.

### Step 3. Root access to the container (optional)

In case you need the root access (e.g., when installing new libaries), you can use the following command to get a root terminal when the container is running

{% include alert terminal='docker exec -u 0 -it graic_con /bin/bash' %}

</div>
