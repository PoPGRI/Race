---
layout: page
title: Description
permalink: /description/
main_nav: false
---

_Post-Perception General Racing Intelligence (PoPGRI)_ is a framework for developing intelligent racing agents that can work with different vehicles and environments. We are developing PoPGRI with the aim of running a competition at [CPS-IOT Week 2021](https://cps-iot-week2021.isis.vanderbilt.edu/). 

The goal of this competition is to record and help advance the state-of-the-art in (model-based and model-free) controller synthesis as applied to tactical decision making for racing environments. Comparing performance of different synthesis approaches is notoriously difficult. There are many computing platforms, approaches, no standard interfaces and problem specifications, and repeatability in research is complicated. We hope that this competition will jumpstart a solution to this problem.

The participants will be required to submit a piece of code (the synthesizer) for generating agents or controllers for vehicles running in partially known, complex, simulated environments. The input to the synthesized controller will come from a perception oracle that will provide a local view of obstacles and the next sequence of gates and goals in the local coordinates of the vehicle in the current environment. The output of the synthesized controller will drive the plant (e.g., a quadcopter, or a fixed-wing aircraft). The synthesizer will take as input (a) vehicle specs: some prior knowledge or a black-box executable for simulating the plant/vehicle including the input and output interfaces and (b) track specs: parameters defining the environment, such as speed of dynamic obstacles, curvatures, etc. With this information, the synthesizer will generate the controller which when combined with the plant and the environment will give a closed system that can be simulated. The performance evaluation will be based on simulations in a set of test environments.

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
