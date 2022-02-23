---
layout: page-fullwidth
title: Running GRAIC -- Multi-Agent(BETA)
date:   2022-04-18T14:25:52-05:00
permalink: installation/multi/
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
**NOTE: Multi-Agent Mode is still under developement, we release this as a beta version. Please help us improve by reporting bugs/optimization at our github issues, thanks!**
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

### Running the GRAIC infrastructure with multi-agent scenario
First, you need to switch to the multi-agent branch.
{% include alert terminal='cd /home/carla/workspace/graic-workspace/src && git checkout feat/multi_agent' %}

Then, run the following command
{% include alert terminal='. ~/workspace/graic-workspace/devel/setup.bash' %}
{% include alert terminal='roslaunch graic_core carla_multiple.launch model_type:=model_free N:=2' %}

### Running the baseline Controller
First Open a new terminal, and then do the following command.
{% include alert terminal='docker exec -it graic_con /bin/bash' %}
{% include alert terminal='. ~/workspace/graic-workspace/devel/setup.bash' %}
{% include alert terminal='roslaunch graic_core graic_agent_wrapper.launch class_file:="/home/carla/workspace/graic-workspace/src/graic_core/src/baseline.py" role_name:="hero0"' %}

### Running your own Controller
Then Open a new terminal, and do the following command.
{% include alert terminal='docker exec -it graic_con /bin/bash' %}
{% include alert terminal='. ~/workspace/graic-workspace/devel/setup.bash' %}
{% include alert terminal='roslaunch graic_core graic_agent_wrapper.launch class_file:="/home/carla/workspace/graic-workspace/src/graic_core/src/your_own_controller.py" role_name:="hero1"' %}

</div>
