---
layout: default
title: Dave Object Models
parent: Underwater Object Models
grand_parent: Dave Models
nav_order: 1
---


As part of the dave project the team has integrated a variety of object models intended to be useful for composing underwater scenarios to reflect common deepsea robot tasks such as search and manipulation.  The dave-specific object models are stored in two places:

1. Within the [dave_object_models](https://github.com/Field-Robotics-Lab/dave/tree/master/models/dave_object_models) ROS package that is contained within the dave repository.
2. Within the [DAVE collection](https://app.ignitionrobotics.org/Cole/fuel/collections/DAVE) of the Ignition Fuel online model repository.

The following example makes use of an example world that demonstrates including models from both locations in a demonstration scenario.  Note that it may take a bit of time to load this demo the first time you run it because of the time taken to download the models from Ignition Fuel.

```
roslaunch dave_demo_launch dave_models.launch
```

Which is expected to start a world that looks like this.

![Screenshot from 2022-02-25 09-29-47](https://user-images.githubusercontent.com/8921143/155760921-53a92b92-898d-428f-b09f-d330780fc5ce.png)


For a full list of the models included see the [dave_ocean_models.world](https://github.com/Field-Robotics-Lab/dave/blob/master/models/dave_worlds/worlds/dave_ocean_models.world).

Below we describe some of the categories of models:

### Torpedos: Mk-46 (lightweight) and Mk-48 (heavyweight)

![Screenshot from 2022-02-25 09-35-30](https://user-images.githubusercontent.com/8921143/155761200-8f31bf5c-40b2-4739-9e83-c50f600ecada.png)

### Unexploded Ordnance

![Screenshot from 2022-02-25 09-40-29](https://user-images.githubusercontent.com/8921143/155761872-8ded3866-dc03-496d-a9c6-728f8a0a1455.png)

### Deepsea floatation - hardhats and 17" glass spheres

![Screenshot from 2022-02-25 09-41-47](https://user-images.githubusercontent.com/8921143/155762079-7e21fdf7-e065-4db0-9aa1-e8b1598c9a52.png)


### Sonobuoy

![Screenshot from 2022-02-25 09-39-30](https://user-images.githubusercontent.com/8921143/155761620-dd698014-6ed0-4eb5-8fd0-77f3ff7952d5.png)


### Flight data recorder

Based on http://uasc.com/home/shop/avionics/cvr-fdr  See pdf procure for images and dimensions

![Screenshot from 2022-02-25 09-36-36](https://user-images.githubusercontent.com/8921143/155761299-7d1afb78-3beb-40c2-bb02-8b2d237f59c7.png)

### MBARI MARS Ocean Observing Node

![Screenshot from 2022-02-25 09-42-56](https://user-images.githubusercontent.com/8921143/155762176-ba45ec52-9001-4d24-8349-59edc44f57a9.png)


### Amphora and Niskin Bottle from Ignition Fuel

![Screenshot from 2022-02-25 09-33-48](https://user-images.githubusercontent.com/8921143/155760988-38daf204-859c-43b6-8614-4717287ca2b9.png)

### A Lionfish from Ignition Fuel

The unofficial mascot of project dave.

![Screenshot from 2022-02-25 09-44-19](https://user-images.githubusercontent.com/8921143/155762402-4dab6602-f19f-4ad1-a333-64b6f63f1293.png)

