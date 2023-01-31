---
layout: page-fullwidth
title: Installing GRAIC locally
date:   2022-04-18T14:25:52-05:00
permalink: installation/local/
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

# Dependencies
- Carla 0.9.13
- Scenario Runner 0.9.13
- Python3.7+

# Installing the Carla 0.9.13 simulator
The official installation guide can be found at [https://carla.readthedocs.io/en/latest/start_quickstart/#carla-installation](https://carla.readthedocs.io/en/latest/start_quickstart/#carla-installation).
A simplified walkthrough is provided below.

**Step 1**
Download the Carla 0.9.13 simulator at [https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.13.tar.gz](https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.13.tar.gz).

**Step 2**
Extract it into a favored path (ie. `/home/CARLA_0.9.13`)

**Step 3**
Run `sudo apt install libomp5` as per this [git issue](https://github.com/carla-simulator/carla/issues/4498).

**Step 4.1**
Add the Carla API to your python path by appending the following lines to your bashrc file (`~/.bashrc`).
Replace `PATHTOCARLA` with the path used in Step 2.

```
export PYTHONPATH=$PYTHONPATH:PATHTOCARLA/PythonAPI/carla/
export PYTHONPATH=$PYTHONPATH:PATHTOCARLA/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg
```

**Step 4.2**
Source the bashrc file by running `source ~/.bashrc`.

**Step 5**
Install pygame by using `python3 -m pip install --user pygame numpy`

**Step 6**
Open the simulator using the following commands.
Replace `PATHTOCARLA` with the path used in Step 2.

```
cd PATHTOCARLA
./CarlaUE4.sh
```

If all the above steps have run correctly, you should see something like in the below image.

<img src="{{site.urlimg}}Carla0913.png">

# Install GRAIC Customized Maps for Carla 0.9.13
The official installation guide can be found at [https://carla.readthedocs.io/en/0.9.13/tuto_M_add_map_package/](https://carla.readthedocs.io/en/0.9.13/tuto_M_add_map_package/).
A simplified walkthrough is provided below.

**Step 1**
Download the 5 raw maps from [https://drive.google.com/drive/folders/13xQbCmlUFVmyqN-elPB8LHSIBsaeLbnZ?usp=sharing](https://drive.google.com/drive/folders/13xQbCmlUFVmyqN-elPB8LHSIBsaeLbnZ?usp=sharing).

**Step 2**
Place the 5 tar.gz files in the corresponding import path (ie. `PATHTOCARLA/import`)

**Step 3**
Import the map using the following:
```
cd PATHTOCARLA
./ImportAssets.sh
```

If all the above steps have run correctly, you should now be able to switch to any of the maps.
To do this, run the following commands:
```
cd PATHTOCARLA
./CarlaUE4.sh
```
Open a new terminal and run the following.
```
cd PATHTOCARLA/PythonAPI/util
python3 config.py --list
```
You should now see 5 maps imported named `Shanghai`, `t1`, `t2`, `t3`, and `t4`.

<img src="{{site.urlimg}}listMaps.png">

You can switch to any of the maps using the following command and replacing `PATHTOMAP` with the path to the maps shown by `python3 config.py --list`.

```
python3 config.py -m PATHTOMAP
```
<img src="{{site.urlimg}}shanghaiTrack.png">

# Installing GRAIC Customized Scenario Runner
<span style="color:red">We are using a customized Scenario Runner, so please do NOT pull from the official SR repo, but USE our GRAIC repo</span>

**Step 1**
Download the CUSTOMIZED 0.9.13 Scenario Runner from the GRAIC public repo [https://github.com/PoPGRI/scenario_runner](https://github.com/PoPGRI/scenario_runner).

**Step 2**
Extract Scenario Runner into your favored path (ie. `/home/scenario_runner-0.9.13`)

**Step 3**
Install Scenario Runner Dependencies using the following commands. Replace `PATHTOSR` with the path used in Step 2.
```
cd PATHTOSR
python3 -m pip install -r requirements.txt
```

**Step 4.1**
Add the Scenario Runner API to your own python path by appending the following line into your bashrc file( `~/.bashrc`).
Replace `PATHTOSR` with the path used in Step 2.

```
export PYTHONPATH=$PYTHONPATH:PATHTOSR
```

**Step 4.2**
Source the file by running `source ~/.bashrc`

**Step 5**
Check the installation by running `from scenario_runner import ScenarioRunner` in Python.
If you have completed the above steps correctly, then there should be no import errors.

# Installing GRAIC Customized Scripts

**Step 1**
Download GRAIC infrastructure scripts for 2022 at [https://github.com/PoPGRI/Race/tree/main](https://github.com/PoPGRI/Race/tree/main).

**Step 2**
Place the folder into your preferred path (ie. `/home/Race`)

**Step 3**
Implement your controller in `agent.py`.
The inputs are the Ground Truth Perception Data.

**Step 4**
You are now ready to launch GRAIC.

*Step 4.1*
(Terminal 1) Launch the simulator
```
cd PATHTOCARLA
./CarlaUE4.sh
```

*Step 4.2*
(Terminal 2) Change the map. In this example we will use the Shanghai map.
```
python3 config.py -m /Game/map_package/Maps/shanghai_intl_circuit/shanghai_intl_circuit
```

*Step 4.3*
(Terminal 3) Run `python3 wrapper.py`.
This should be in the `/Race/` folder.

*Note*: you may use `pkill` to kill python if the car dies and the scenario keeps running.
Check the code in `wrapper.py` to enable/disable scenarios to help with debugging

**Additional comments**
* Carla Python API: [https://carla.readthedocs.io/en/0.9.13/python_api/](https://carla.readthedocs.io/en/0.9.13/python_api/)
* Model + Track:
The code released only runs the Tesla Model 3 on the Shanghai Track, but during testing we will use different models + different race tracks.

# Simple Controller Demo

A simple controller demo can be found at [https://drive.google.com/file/d/1u4rj-sxMd6fuTnt_hS9PL_gUbfWdRjyO/view?usp=sharing](https://drive.google.com/file/d/1u4rj-sxMd6fuTnt_hS9PL_gUbfWdRjyO/view?usp=sharing).


</div>
