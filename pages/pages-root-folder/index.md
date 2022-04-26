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

## Description

_Generalized RAcing Intelligence Competition (GRAIC)_ is a simulated vehicle race affiliated with [CPS-IOT Week 2022](https://cpsiotweek.neslab.it/). GRAIC aims to bring together researchers in AI, planning, synthesis, and control to create a platform for comparing different algorithms for controlling vehicles in dynamic and uncertain environments. We hope that it will also be fun.

**GRAIC '22 vs. GRAIC '21**
This year, we hope to improve GRAIC to encourage wider participation.
We now have an AWS image that lowers the hardware entry requirements.
Instructions for using the AWS image can be found on our [installation page](https://popgri.github.io/Race/installation/).
We also aim to have multi-agent racing and an improved testing pipeline.
Please continue to check our webpage for updates!

[**Join our mailing list!**](https://docs.google.com/forms/d/e/1FAIpQLSesyCan0-i0r3mhxe21l4YEDFNLiItINRJz9qEoYrI8jQ04Mg/viewform?usp=sf_link)
If you are interested in participating in GRAIC or just want to stay up to date, please fill out this [form](https://docs.google.com/forms/d/e/1FAIpQLSesyCan0-i0r3mhxe21l4YEDFNLiItINRJz9qEoYrI8jQ04Mg/viewform?usp=sf_link) and we will add you to our mailing list.
Updates can also be found at the bottom of this page.


### Tl;dr

We are providing a simulation environment, test vehicles, tracks, scoring function, and documentation. As a competitor, you will use the given API and develop your racing controller. In early May, you will submit your racing controller code. The competition tracks will be different from the testing tracks. Multiple vehicles will be involved. We will run the races with your controllers and provide results, data, video feedback, and announce winners during CPSWeek. There will be different race categories and prizes.


### Details

At runtime, the input to the controller will come from a _perception oracle_ that will provide as input a local view of obstacles, lanes, and gates on the track near the vehicle. The tracks will have à priori unknown static and moving obstacles. The outputs from the controller (brake, throttle, and steering) will drive the vehicle. In some race categories, you will be provided a mathematical vehicle model, and in other categories you will be provided a black-box vehicle simulator. The perception and control interfaces will not change. Read the [Docs](https://popgri.github.io/Race/documentation/) page for more details about tracks, obstacles, vehicles, and APIs.

### Highlights

* **New this year!** Multi-agent races to compete against other submitted constrollers
* GRAIC focuses on decision, control, planning, and safety, and therefore, we will provide a perception oracle and related API
* Your controller will run races across multiple vehicles and tracks
* Different race categories for model-free and model-based vehicles

## Contact Information

If you have any questions regarding the competition or simulator framework, please contact us at <a href="mailto:graic2021@gmail.com">graic2021@gmail.com</a>.
If you fill out this [form](https://docs.google.com/forms/d/e/1FAIpQLSesyCan0-i0r3mhxe21l4YEDFNLiItINRJz9qEoYrI8jQ04Mg/viewform?usp=sf_link), we will add you to our mailing list.

## Important Dates

*GRAIC 2022 is still under construction, all dates are tentative. Please check back often for updated dates.*

- **January 26**: Single agent platform beta released
- **February 21**: Multi-agent platform beta released
- **January - early March**: Feedback to participants and platform updates
- **March 31**: Final GRAIC-22 platform release
- **April 8-9**: [Engineering Open House Presentation](https://www.eohillinois.org/)
- **April 18: Submissions open**
- **May 1: Submissions close, final races held**
- **May 3-6**: [CPS-IOT Week 2022](https://cpsiotweek.neslab.it/), final results and live GRAIC event


## Updates

<ul>
<li><b>04/18/2022</b> - The submission page is now open! Please fill out <a href = "https://docs.google.com/forms/d/e/1FAIpQLSecJQCAh5MSgGkYo__-aVQgSEl8dEkxR8_VvZt7PmkIkQCnaA/viewform?usp=sf_link">this form</a> to register and submit.</li>
</ul>
<li><b>01/25/2022</b> - Our AWS and docker betas are now available! This lowers the hardware requirements. Please go to our <a href = "https://popgri.github.io/Race/installation/">Getting started</a> page for more information!</li>
</ul>


<ul style="color:gray">
  <li><i>05/03/2021</i> - Pre-submission. You can pre-submit your controllers to ensure that it works in our finalized framework. Please upload your controllers to the google form provided.</li>

  <li><i>05/03/2021</i> <a href = "https://github.com/PoPGRI/Race/releases/tag/0.2.1">GRAIC Beta Release v0.2.1</a> We have updated several things in this release. This is the release we plan to use in the competition, so please give us feedback.</li>

  <li>CPS Week registration is now open! Registration is $10. When you register for CPS Week, please choose the "Generalized RAcing Intelligence Competition (GRAIC)" option to sign up.</li>

  <li><i>03/30/2021</i> - <a href = "https://github.com/PoPGRI/Race/releases/tag/0.1.2">GRAIC Beta Release v0.1.2</a> We have updated the CARLA docker image.</li>

  <li><i>03/10/2021</i> - <a href = "https://github.com/PoPGRI/Race/releases/tag/0.1.1">GRAIC Beta Release v0.1.1</a> We have modified LaneInfo message type to include more information about left and right lanes, and fixed disjoint lane markers at junctions of road segments.</li>

  <li><i>03/01/2021</i> - We have released our <a href = "https://github.com/PoPGRI/Race/releases/tag/0.1.0">beta</a>! Please go to the <a href = "https://popgri.github.io/Race/installation/">Getting started</a> page to download. We invite all feedback.</li>

</ul>

## Citing GRAIC

If you would like to cite us, please use the following Bibtex entry.

{% highlight bibtex %}
@misc{GRAICrace,
      title        = "GRAIC: A simulator framework for autonomous racing",
      author       = "{Minghao Jiang and Zexiang Liu and Kristina Miller and Dawei Sun and Arnab Datta and Yixuan Jia and Sayan Mitra and Necmiye Ozay}",
      howpublished = "\url{https://popgri.github.io/Race/}",
      year         = 2021
    }
{% endhighlight %}


</div><!-- /.medium-8.columns -->
</div><!-- /.row -->
