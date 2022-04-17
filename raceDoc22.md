# GRAIC 2022 Racing Document

## Race Description

Each race is defined by its configuration.

A racing configuration is defined by:
a race type (single vs. multi-agent), (2) a vehicle type, (3) a track (environment), and (4) scenarios.

In this document, we will describe each part of the configuration, the rules for our final race, and the submission requirements.

### Single-agent track

In the single agent track of our competition, we will run multiple races. The scores from these races are then weighted based off of the difficulty of the race, and then overall winner is chosen using the weighted scores.

### Multi-agent track

In the multi-agent track of our competition, each submitted controller will have the opportunity to compete against every other controller.
Then, based off these results, the submitted controllers will compete in a bracket style tournament to determine the overall winner of the multi-agent competition.

## Vehicle types

There are two tracks that teams can participate in: model-agnostic and model-based. In the model-agnostic category, complicated vehicle physics is given by CARLA’s vehicle simulator. In the model-based option, the vehicle will follow the dynamics detailed on our [vehicle models page](https://popgri.github.io/Race/documentation/models/).

### Model-agnostic vehicles

Below is a table that provides a superset of vehicles that may be used in the model-agnostic track of the competition.

**Vehicle Name**|**Vehicle ID**
:-----:|:-----:
BMW Isetta| vehicle.bmw.isetta
Tesla Cybertruck| vehicle.bmw.cybertruck
Mercedes Benz Coupe| vehicle.mercedes-benz.coupe
Kawasaki Ninja| vehicle.kawasaki.ninja
Dodge Charger Police Car| vehicle.dodge\_charger.police
Tesla Model 3| vehicle.tesla.model3
Volkswagen T2| vehicle.volkswagen.t2
BH Crossbike| vehicle.bh.crossbike
Mini Cooper| vehicle.mini.cooperst

### Model-based vehicles

Below is the table with the parameters that will be used for the model-based track.

**m**|**f<sub>1</sub>**|**f<sub>2</sub>**|**f<sub>3</sub>**|**a**|**b**|**C<sub>af</sub>**|**C<sub>ar</sub>**|**I<sub>z</sub>**|**Vehicle ID**
:-----:|:-----:|:-----:|:-----:|:-----:|:-----:|:-----:|:-----:|:-----:|:-----:
1800| 40.59| 0.01| 74.63| 1.2| 1.65| 1.4e5| 1.2e5| 3270| vehicle.model\_based.1

## Track Configuration

There are 5 tracks we created for the competition, all 5 maps are already pre-installed in the docker image. And we will evaluate your performance on all 5 tracks.

## Scenario Configuration
In single-agent mode, we will test your controller with certain scenarios. We have provided a sample scenario to you in the docker; however, please note that the scenario during the actual testing time might be different from what you see in the docker.

In multi-agent mode, we will not use any scenario, the only other actor you will be competing against is other participant’s controller.

## Submission Requirements

The submission site opens on April 18, and can be found at [https://graic2022submit.web.illinois.edu/graic2022](https://graic2022submit.web.illinois.edu/graic2022).

Each team must create an account with us to be able to submit. Please fill out our [form](https://forms.gle/SF6ffppeuYJp5w5z8) so that we can create your account. Teams may resubmit controllers up to the deadline. In the days leading up to the race, we will run your controllers on our test tracks and provide videos and race logs as feedback. We will also maintain a leaderboard of the test tracks so teams know how they compare against each other. **PLEASE NOTE THAT THE TEST TRACKS ARE ONLY A SUBSET OF THE FINAL RACING CONFIGURATIONS, AND TEAMS SHOULD BE AWARE AS TO NOT OVERFIT TO OUR TESTING SCENARIOS.** After the submission site closes on **May 1**, we will run the last controllers submitted by the teams on our final race configurations, and we will reveal the results of the race at our CPS-IOT Week presentation.

## File Structure

What you will submit is a single zip file named **controller.zip** that contains all your code. The entry must be in the top level named **user_controller_file.py**, and it must be able to be executed with one single command (`python3 agent_wrapper.py ego_vehicle`) without additional arguments as is listed in the instructions; we will be using the original `agent_wrapper.py` to overwrite yours if you provide any. If you install any python package(E.g. `python3 -m pip install XXX`) in our docker image, you must list that in **requirement.txt**; if you install any apt packages in our docker image(E.g. `sudo apt install XXX`), you must list that in **apt.txt**.


The file structure should be:

- controller.zip
  - main.py
  - requirement.txt
  - apt.txt
  - subfolder (if any)
    - other_file.py (if any)

      …
      …

If you have any questions, please email us at graic2021@gmail.com or post to our [forum](https://groups.google.com/g/graic21). Good luck and happy racing!
