---
layout: page-fullwidth
title: Running GRAIC -- Model-based
date:   2022-04-18T14:25:52-05:00
permalink: installation/model-based/
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

---   
The whole racing system consists of three independent programs, the CARLA simulator, the GRAIC infrastructure, and your controller, which should be started in three terminals. If you are using Docker, you can get a new terminal connecting to the running container by running
{% include alert terminal='docker exec -it graic_con /bin/bash' %}
where `graic_con` is the name of the running container.

### Running the CARLA simulator
If you are using Docker, run the following command in a new terminal
{% include alert terminal='~/workspace/carla-simulator/CarlaUE4.sh -opengl' %}

If you are using AWS, run the following command in a new terminal
{% include alert terminal='~/workspace/carla-simulator/CarlaUE4.sh' %}

If everthing works, you shoule be able to see the CARLA window.

### Running the GRAIC infrastructure with model based vehicles
First, you need to switch to the model-based branch.
{% include alert terminal='cd /home/carla/workspace/graic-workspace/src && git checkout feat_model_based' %}

Then, run the following command
{% include alert terminal='. ~/workspace/graic-workspace/devel/setup.bash' %}
{% include alert terminal='roslaunch graic_core graic_single.launch synchronous_mode_wait_for_vehicle_control_command:=False' %}

### Running the baseline Controller
This procedure is the same as running model free baseline. However, please note that the baseline controller we offered is only tuned for model-free vehicles, so it might now work so well with model-based vehicles.

### Driving with manual control for fun
When the pygame window pops up, press b and then you should be able to drive using wasd.

</div>
