---
layout: page
title: Description
permalink: /description/
main_nav: false
---

_Post-Perception General Racing Intelligence (PoPGRI)_ is a simulation framework for developing intelligent racing agents that can work with different vehicles and environments. PoPGRI will be released as a part of a synthesis competition at [CPS-IOT Week 2021](https://cps-iot-week2021.isis.vanderbilt.edu/) and future robotics and autonomy events. Decision making and controller for raching vehicles is an exicing research topic with many competitive approaches. How can we reliably evaluate the merits of different approahces? Most algorithms and tools arevery specific to vehicles and work only on particular computing platforms.  We hope that PoPGRI will bring the research community together and help arrive at an acceptable answer. 

# The competition
The participants will submit a piece of code (_the synthesizer_) for generating _agents_ for controlling vehicles running in partially known, complex, simulated environments. The input to the agent will come from a _perception oracle_ that will provide a local view of obstacles and the next sequence of gates and goals near the vehicle. The output of the synthesized controller will drive the vehicle. The synthesizer will take as input (a) vehicle specs: some prior knowledge or a black-box executable for simulating the plant/vehicle including the input and output interfaces and (b) track specs: parameters defining the environment, such as speed of dynamic obstacles, curvatures, etc. With this information, the synthesizer will generate the controller which when combined with the plant and the environment will give a closed system that can be simulated. The performance evaluation will be based on simulations in a set of test environments.

## Evaluation criteria: 

The competition will be based on running the synthesizer and the synthesized controller in a number of different environments and with different vehicles. The following metrics will be used: 
1. Course completion speed, 
2. Safety violations in stress environments, 
3. Computational load on synthesized controller (online), e.g., per unit time, 
4. Computation load for the offline synthesis algorithm, e.g., simulation load. 

Several different test environments and 2-3 models will be used for generating these metrics. Each participating controller will perform the race solitarily for this year’s competition. Simulating multiple agents in complex environments deterministically is a complicated challenge. In the future, we do intend to make the competition a truly multi-agent race.

## Distinguishing features:
* We want this competition to feature on decision making, control, transfer, safety, robustness, and reliable performance, and not on perception. Therefore, we have formulated the synthesis problem defined above with a perception oracle which gives the ground truth around the vehicle. 

* We want to avoid synthesis approaches that are overfitted for specific vehicles. This is an important aspect in which we are different from robotics challenges. Therefore, we abstract the vehicle dynamics with parameters and a black-box simulator. 


## Plan for both in-person and remote possibilities

This year the competition will be entirely simulation-based. We will create a timeline for the following significant milestones leading upto CPSWeek. 
- Information session
- Registration of participants
- Publication of vehicle models/executables, and training tracks, and specifications for the synthesizer
- Publication of performance evaluation code with metrics
- Preliminary submissions for “type checking” entries
- Final submissions, and competition

Technical details about submissions, repositories, docker, VM, etc. will be announced in due course. We plan to learn from and collaborate with organizers of previous competitions for F1/10 (https://cps-vo.org/group/CPSchallenge), ARCH (https://cps-vo.org/group/ARCH/FriendlyCompetition), Neural Network verification (https://sites.google.com/view/vnn20/vnncomp),  and formal synthesis (http://docs.fmrchallenge.org/en/v0.0.4/).
