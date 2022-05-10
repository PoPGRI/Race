---
#
# Use the widgets beneath and the content will be
# inserted automagically in the webpage. To make
# this work, you have to use › layout: frontpage
#
layout: page-fullwidth
header:
  image_fullwidth: "baseline3.gif"

permalink: /index.html
homepage: true
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
### [GRAIC '22 Leaderboard](https://graic-2022.herokuapp.com)

### [GRAIC Forum](https://groups.google.com/u/1/g/graic21)


### [GRAIC '22 Call for Participation](https://popgri.github.io/Race/assets/CFP2022.pdf)
  
### Introduction

_Generalized RAcing Intelligence Competition (GRAIC)_ is a simulated vehicle race. The most recent version of the competition was co-located with [CPS-IOT Week 2022](https://cpsiotweek.neslab.it/). GRAIC aims to bring together researchers in AI, planning, synthesis, and control to create a platform for comparing different algorithms for controlling vehicles in dynamic and uncertain environments. We hope that it will also be fun.

**GRAIC '22 vs. GRAIC '21**
This year, we hope to improve GRAIC to encourage wider participation.
We now have an AWS image that lowers the hardware entry requirements.
Instructions for using the AWS image can be found on our [installation page](https://popgri.github.io/Race/installation/).
We also aim to have multi-agent racing and an improved testing pipeline.
Please continue to check our webpage for updates!

GRAIC provides a simulation environment, test vehicles, tracks, scoring function, and documentation. As a competitor, you will use the given API and develop your racing controller. In early May, you will submit your racing controller code. The competition tracks will be different from the testing tracks. Multiple vehicles will be involved. We will run the races with your controllers and provide results, data, video feedback, and announce winners during CPSWeek. There will be different race categories and prizes.


### Details

At runtime, the input to the controller will come from a _perception oracle_ that will provide as input a local view of obstacles, lanes, and gates on the track near the vehicle. The tracks will have à priori unknown static and moving obstacles. The outputs from the controller (brake, throttle, and steering) will drive the vehicle. In some race categories, you will be provided a mathematical vehicle model, and in other categories you will be provided a black-box vehicle simulator. The perception and control interfaces will not change. Read the [Docs](https://popgri.github.io/Race/documentation/) page for more details about tracks, obstacles, vehicles, and APIs.

* **New in 2022!** Head-to-head races; compete against other submitted constrollers
* GRAIC focuses on decision, control, planning, and safety, and therefore, we will provide a perception oracle and related API
* Your controller will run races across multiple vehicles and tracks
* Different race categories for model-free and model-based vehicles

## Contact 

Email us at <a href="mailto:graic2021@gmail.com">graic2021@gmail.com</a>.
Join mailing list [form](https://docs.google.com/forms/d/e/1FAIpQLSesyCan0-i0r3mhxe21l4YEDFNLiItINRJz9qEoYrI8jQ04Mg/viewform?usp=sf_link).

## Important Dates

*Please check back for the next GRAIC competition updates. Following were the dates for 2022 competition.*

- **January 26**: Single agent platform beta released
- **February 21**: Multi-agent platform beta released
- **January - early March**: Feedback to participants and platform updates
- **March 31**: Final GRAIC-22 platform release
- **April 8-9**: [Engineering Open House Presentation](https://www.eohillinois.org/)
- **April 18: Submissions open**
- **May 1: Submissions close, final races held**
- **May 5, 1-3pm (CT)**: [CPS-IOT Week 2022](https://cpsiotweek.neslab.it/), final results and live GRAIC event. Zoom link to be published


## Updates

<ul style="color:gray"> 
  - <i>05/3/2022</i> 2022 <a href="https://popgri.github.io/Race/outreach/">Live event</a> at CPSWeek.
  - <i>04/18/2022</i> 2022 submission page opens! Fill out <a href = "https://docs.google.com/forms/d/e/1FAIpQLSecJQCAh5MSgGkYo__-aVQgSEl8dEkxR8_VvZt7PmkIkQCnaA/viewform?usp=sf_link">this form</a> to submit.
  - <i>01/25/2022</i> GRAIC AWS and docker available! Visit <a href = "https://popgri.github.io/Race/installation/">Getting started</a> page for details.
  - <i>05/03/2021</i> 2021 pre-submission open. Upload your controllers to the google form provided.
  - <i>05/03/2021</i> <a href = "https://github.com/PoPGRI/Race/releases/tag/0.2.1">GRAIC Beta Release v0.2.1</a>.
  - CPS Week registration open! When you register,  choose  "Generalized RAcing Intelligence Competition (GRAIC)" option.
  - <i>03/30/2021</i> <a href = "https://github.com/PoPGRI/Race/releases/tag/0.1.2">GRAIC Beta Release v0.1.2</a>.
  - <i>03/10/2021</i>  <a href = "https://github.com/PoPGRI/Race/releases/tag/0.1.1">GRAIC Beta Release v0.1.1</a> LaneInfo message type now has more info
  - <i>03/01/2021</i> GRAIC <a href = "https://github.com/PoPGRI/Race/releases/tag/0.1.0">beta</a> released! Visit <a href = "https://popgri.github.io/Race/installation/">Getting started</a> page to download.
</ul>

## Citing GRAIC

Please cite GRAIC as:

{% highlight bibtex %}
@misc{GRAICrace,
      title        = "GRAIC: A simulator framework for autonomous racing",
      author       = "{Minghao Jiang and Zexiang Liu and Kristina Miller and Dawei Sun and Arnab Datta and Yixuan Jia and Sayan Mitra and Necmiye Ozay}",
      howpublished = "\url{https://popgri.github.io/Race/}",
      year         = 2021
    }
  
@misc{GRAIC-CI-OCAR21,
      title        = "Continuous Integration and Testing for Autonomous Racing Software: An Experience Report from GRAIC",
      author       = "{Minghao Jiang and Kristina Miller and Dawei Sun and Zexiang Liu and Yixuan Jia and Arnab Datta and  Necmiye Ozay and Sayan Mitra}",
      howpublished = "Presented at the Workshop on Opportunities and Challenges in Autonomous Racing (2021) at IEEE ICRA",
      url = "https://par.nsf.gov/servlets/purl/10296575",
      year         = 2021
    }  
{% endhighlight %}


</div><!-- /.medium-8.columns -->
</div><!-- /.row -->
