---
layout: page-fullwidth
title: Installing GRAIC on AWS
date:   2022-04-18T14:25:52-05:00
permalink: installation/aws/
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

## Requirements
- An AWS account
- A NoMachine client, which can be downloaded from [https://www.nomachine.com/](https://www.nomachine.com/)

<span style="color:red">
Note that the AWS has some costs associated with it.
We recommend to use a local environment to develop the code and only use the EC2  instance to test your code.
Don’t forget to turn off your instance when you are not working; otherwise timer will keep counting and charge you.
You can check out more info on pricing at these links:
</span>
<ul>
<li><a href="https://aws.amazon.com/ec2/instance-types/g4/">https://aws.amazon.com/ec2/instance-types/g4/</a></li>
<li><a href="https://aws.amazon.com/ec2/pricing/on-demand/">https://aws.amazon.com/ec2/pricing/on-demand/</a></li>
</ul>


## Video Tutorial
(A written tutorial is included after this video tutorial)

<span style="color:red">Note that Step 1.1 (Choose AMI) is outdated. Please review Step 1.1 and return to the video for Step 1.2. Additionally, Step 1.8 (Setting a password for Ubuntu) is not included in this video tutorial. After Step 1.7 (Review and Launch), please perform Step 1.8, and return to the video for Step 2 (GUI via NoMachine).</span>

<center>
<iframe width="560" height="315" src="https://www.youtube.com/embed/nqaC1ZXb4H0" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</center>

## Step 1. Creating an EC2 Instance
- Go to your AWS EC2 dashboard. On the top right corner, change the region to "US East (N. Virginia) us-east-1". This step is necessary because our AMI is hosted in this region. If your connection to this region is slow and you prefer another region, please contact us, and we will copy the AMI to your preferred region.
- Locate the "Launch instances" button. Click on it and then follow the instructions below.

### Step 1.1 Choose AMI.
Search for "GRAIC2023" and you should be able to find some AMIs in "Community AMIs".
<img src="{{site.urlimg}}ami_Carla23.png">

### Step 1.2 Choose an Instance Type
Choose "g4dn.4xlarge". You can also choose "g4dn.8xlarge" if you need more CPUs.

### Step 1.3 Configure Instance Details
Leave this section unchanged.

### Step 1.4 Add Storage
Leave this section unchanged.

### Step 1.5 Add Tags
Leave this section unchanged.

### Step 1.6 Configure Security Group
Click on "Add Rule" and add port 4000 as shown in the figure. Please note that we set the sources for both ports to "Anywhere" for simplicity, but it would be safer to restrict the sources to a smaller set, for example, the subnet owned by your university.
<img src="{{site.urlimg}}portconfig.png">

### Step 1.7 Review and launch
Finally, click on the "Launch" button which should appear at the lower right corner.
<img src="{{site.urlimg}}launch.png">

A pop-up window should appear and ask you to select an existing key pair. If you don't have an existing key pair, select "Create a new key pair", give it a name, and "Download Key Pair". Then "Launch Instances".
<img src="{{site.urlimg}}aws_key.png">

It might take several minutes to set up. Locate the new instance in your EC2 dashboard, and you can find its Public IPv4 address there.

### Step 1.8 Setting a password for the user "ubuntu"
You may want to set a strong password for the main user "ubuntu". To do that, select the new instance in your EC2 dashboard and click on "Connect" which should appear at the upper right corner.
<img src="{{site.urlimg}}instancerunning.png">

Then you should be redirected to the page as shown in the figure. Change the username to "ubuntu" and click on "Connect".
<img src="{{site.urlimg}}connect.png">

You should be able to see a new window which looks like the following picture. Change your password by running command
{% include alert terminal='sudo passwd ubuntu' %}
 and following the prompts.
<img src="{{site.urlimg}}webterminal.png">

## Step 2. GUI via NoMachine
Open NoMachine, click on "Add" which appears at the upper left corner.
<img src="{{site.urlimg}}nomachine_login.png">

Choose whatever name you like, and fill "Host" with your instance's Public IPv4 address. Set "Port" to 4000 and use the NX protocol.
<img src="{{site.urlimg}}nomachine_address.png">

 Click on "Configuration" on the left. Select "Use key-based authentication with a key you provide" and click on the "Modify" button. Then, select the key file downloaded in Step 1.7. Go back and click on "Connect". If it says "key is missing", click on the "Modify" button, select the key you downloaded, and try again. This might be a bug of NoMachine.
<img src="{{site.urlimg}}nomachine_config.png">

If asked, the username is "ubuntu". After logging into the machine, open a terminal, run
{% include alert terminal='~/scripts/fixCarla.sh' %}
After it finishes, you should be able to use carla by running
{% include alert terminal='~/workspace/carla-simulator/CarlaUE4.sh' %}
All files are installed in the "~/workspace" directory. Enjoy!

</div>
