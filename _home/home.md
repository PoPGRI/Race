---
layout: page
title: Description
permalink: /description/
main_nav: false
---

_Generalized RAcing Intelligence Competition (GRAIC)_ is a simulation framework for developing intelligent racing agents that can work with different vehicles and environments. GRAIC will be released for a synthesis competition at [CPS-IOT Week 2021](https://cps-iot-week2021.isis.vanderbilt.edu/) and possibly will continue with future robotics and autonomy events. Decision making and control for racing has many competing approaches. How can we reliably evaluate the merits of different algorithms, independent of the ever-improving computing and sensing modules? Most algorithms and tools are hyper-customized to vehicles. Some only work on particular computing platforms.  We hope that GRAIC will bring the research community together and help arrive at an acceptable answer.

# The competition
The participants will submit a piece of code (_the synthesizer_) for generating _agents_ for controlling vehicles. This agent will then control the vehicle in a partially known, complex, simulated _track_. At runtime, the input to the agent will come from a _perception oracle_ that will provide as input a local view of obstacles and gates on the track near the vehicle. The outputs from the agent will drive the vehicle. The synthesizer will take as input (a) black-box executable for simulating the vehicle in an empty track, (b) track parameters, such as speed of dynamic obstacles, curvatures, etc.

## Evaluation:

The competition will be based on running the synthesizer and the synthesized agent in a number of different tracks and with different vehicles. The metrics will be for example, course completion speed, safety violations, computational load on agent, simulation load, etc.  

Several different test environments and 2-3 models will be used for generating these metrics. Each participating controller will perform the race solitarily for this year’s competition. Simulating multiple agents in complex environments deterministically is a complicated challenge. In the future, we do intend to make the competition a truly multi-agent race.

## Features:
* We want this competition to focus on decision making, control, transfer, safety, robustness, and reliable performance, and not so much on perception. Hence, the perception oracle.

* We want to go beyond approaches that are overfitted for specific vehicles and tracks. Hense, the offline generation of the agent and the online performance evalulation.


## Plan for both in-person and remote possibilities

This year the competition will be entirely simulation-based. We will create a timeline for the following significant milestones leading upto CPSWeek.
- Information session
- Registration of participants
- Publication of vehicle models/executables, and training tracks, and specifications for the synthesizer
- Publication of performance evaluation code with metrics
- Preliminary submissions for “type checking” entries
- Final submissions, and competition

Technical details about submissions, repositories, docker, VM, etc. will be announced in due course. We plan to learn from and collaborate with organizers of previous competitions for [F1/10](https://cps-vo.org/group/CPSchallenge), ARCH (https://cps-vo.org/group/ARCH/FriendlyCompetition), [Neural Network verification](https://sites.google.com/view/vnn20/vnncomp),  and [formal synthesis](http://docs.fmrchallenge.org/en/v0.0.4/).
