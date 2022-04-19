---
layout: page-fullwidth
title: Racing environments and scoring
main_nav: true
date:   2022-04-18T14:25:52-05:00
permalink: documentation/tracks/
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

We are providing several test environments with maps, other vehicles, pedestrians, and static obstacles that the ego vehicle must avoid. We are also providing a scoring function.

# Maps

These test environments will be different than the ones used during the final race.
Below are two of the maps that are included with our beta release.

<img src="{{site.urlimg}}track1.png">
<img src="{{site.urlimg}}track2.png">

# Scenarios

Scenarios will be constructed from vehicles and pedestrians.
These vehicles and pedestrians can be either static or dynamic.
Some example scenarios that the ego vehicle can encounter are a pedestrian crossing the street or congested traffic.

# Evaluation

The evaluation functions is roughly the following:

The time that it takes to pass each milestone waypoint is added to the total score. There will be some penalty added for every collision or when the vehicle runs out of bounds.
The winner of the competition will be the average lowest score over multiple scenarios.

</div>