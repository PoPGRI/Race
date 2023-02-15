---
layout: page
title: GRAIC 3.0 Call for Participation
header:
  image_fullwidth: "yan_demo.gif"
permalink: cfp23/
---

Dear Colleague,

We invite you to participate in the 3rd Generalized RAcing Intelligence Competition (GRAIC) which is co-located with CPSWeek 2023.
Please help disseminate this call for participation through suitable channels. This can be a nice project for a small team of 2-3 undergraduate students working over a month. Graduate students can also explore research around this theme.

Generalized RAcing Intelligence Competition (GRAIC) is a simulated vehicle race co-located with CPS-IOT Week 2023. GRAIC brings together researchers in AI, planning, synthesis, and control to create a platform for comparing different algorithms for controlling vehicles in dynamic and uncertain environments.

This year, we have an improved GRAIC infrastructure to encourage wider participation. We now have an AWS image that lowers the hardware entry requirements. Instructions for using the AWS image can be found on our installation page. We also aim to have multi-agent racing and an improved testing pipeline. Please continue to check our webpage for updates!

GRAIC provides a simulation environment, test vehicles, tracks, scoring function, and documentation. As a competitor, you will use the given API and develop your racing controller. In early May, you will submit your racing controller code. The competition tracks will be different from the testing tracks. Multiple vehicles will be involved. We will run the races with your controllers and provide results, data, video feedback, and announce winners during CPSWeek. There will be different race categories and prizes.

### Details

At runtime, the input to the controller will come from a perception oracle that will provide as input a local view of obstacles, lanes, and gates on the track near the vehicle. The tracks will have a priori unknown static and moving obstacles. The outputs from the controller (brake, throttle, and steering) will drive the vehicle. In some race categories, you will be provided a mathematical vehicle model, and in other categories you will be provided a black-box vehicle simulator. The perception and control interfaces will not change. Read the Docs page for more details about tracks, obstacles, vehicles, and APIs. GRAIC features:

  - Head-to-head races; compete against other submitted controllers
  - GRAIC focuses on decision, control, planning, and safety, and therefore, we will provide a perception oracle and related API
  - Your controller will run races across multiple vehicles and tracks
  - Different race categories for model-free and model-based vehicles

### Contact

Email us at graic2021@gmail.com. Join our mailing list form.

### Important Dates

Please check our website for updates.

  - January 31: Beta platform release
  - February 16: Info session and beta feedback
  - March 17: Final platform release, pre-submission opens
  - March 31: Final submission page opens
  - May 2: Submission closes, final races held
  - May 9: CPS-IOT Week 2023, final results and live event

We look forward to your participation!
