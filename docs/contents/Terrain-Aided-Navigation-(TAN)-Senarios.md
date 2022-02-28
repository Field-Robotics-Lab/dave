---
layout: default
title: TAN Scenarios
nav_exclude: true
---

# TAN Development support features
Introduces TAN development scenarios using Dave project incorporating various sensors and bathymetry models.

## Table of Contents
<!-- TOC generated with https://github.com/ekalinin/github-markdown-toc -->
<!--
 cat fls_model_standalone.md | ./gh-md-toc -
-->
* [Definitions](#Definitions)
* [Interfaces](#Interfaces)
* [Feature Senarios](#Feature-Senarios)
  * [1. Dead Reckoning](#1-dead-reckoning)
    * [1.1 Inertial Navitation System (Utilizing IMU/Pressure Sensors)](#11-inertial-navitation-system-utilizing-imupressure-sensors)
    * [1.2 Utilizing DVL Sensor](#12-utilizing-dvl-sensor)
    * [1.3 Ultra-Short Baseline](#13-ultra-short-baseline)
  * [2. Positional Error Reset](#2-Positional-Error-Reset)
    * [2.1 Utilizing GPS sensor](#21-utilizing-gps-sensor)
  * [3. Feature-based Navigation](#3-Feature-based-Navigation)
    * [3.1 Utilizing Multibeam Echosounder](#31-utilizing-multibeam-echosounder-for-forward-looking-sonar-fls-images)
    * [3.2 Utilizing DVL Sensor for Bathymery gradient estimation](#32-utilizing-dvl-sensor-for-bathymery-gradient-estimation)
  * [4. Terrain Feature Set](#4-Terrain-Feature-Set)
    * [4.1 Incorporating Bathymetry Model](#41-Incorporating-Bathymetry-Model)
  * [5. Evaluation](#5-Evaluation)
    * [5.1 Extracting vehicle position using Gazebo service call](#51-Extracting-vehicle-position-using-Gazebo-service-call)
***


## Definitions
- TAN uses an existing map of the seafloor and estimates the position of the vehicle on the map by matching bathymetry measurements from vehicle sensors with locations on the map with the same altitude (given operation depth). [Melo, J., & Matos, A. (2017). Survey on advances on terrain based navigation for autonomous underwater vehicles. Ocean Engineering, 139, 250-264.](https://doi.org/10.1109/AUV50043.2020.9267886)
- TAN is a sensor-based navigation technique that bounds the error growth of dead-reckoning using a map with terrain information, provided that there is enough terrain variability. An obvious advantage of Terrain Based Navigation is the fact that no external aiding signals or devices are required. [Osterman, N., & Rhén, C. Exploring the sensor requirements for particle filter-based terrain-aided navigation in AUVs. In 2020 IEEE/OES Autonomous Underwater Vehicles Symposium (AUV)(50043) (pp. 1-6). IEEE.](https://doi.org/10.1016/j.oceaneng.2017.04.047)

## Interfaces
![resources](https://docs.google.com/drawings/d/e/2PACX-1vQw6IixHBrj8z209umBjlr__jfWUeddNlnGIzbkpk9CeKo7XlRldaamlnnY-fu6GKDF1dSf3oGDGBWY/pub?w=960&h=720)

## Feature scenarios

### 1. Dead Reckoning

#### 1.1 Inertial Navitation System (Utilizing IMU/Pressure Sensors)
Inertial Navigation system designing can utilize the generic sensor plugins provided by the Gazebo. Both IMU and Pressure sensors are available. There is a simple example of Dead reckoning at the `glider_hybrid_whoi` repo. For details, [Standalone glider deadreckoning](https://github.com/Field-Robotics-Lab/glider_hybrid_whoi/pull/37#issue-596871651). Source code is at [glider_deadreckoning](https://github.com/Field-Robotics-Lab/glider_hybrid_whoi/tree/master/glider_deadreckoning). It is a standalone ROS node python script using a simple algorithm ([this](https://github.com/Field-Robotics-Lab/glider_hybrid_whoi/issues/3#issuecomment-778419590)).

#### 1.2 Utilizing DVL Sensor
The DVL is generally utilized for two purposes: bottom tracking and water tracking. Bottom tracking is commonly used in TAN systems, particularly in dead reckoning. The purpose is to attempt to minimize vehicle deviations from a set heading. The outputs from the DVL can take the form of linear velocities which, when fused with IMU data, can indicate vehicle position in GPS-denied environments. A description of the basic functions of the DVL can be found [here](https://github.com/Field-Robotics-Lab/dave/wiki/whn_dvl_examples) along with examples.

Uses for DVL are expanded within the Dave project via the DVL water tracking and current profiling capabilities. These capabilities allow for the potential addition of water-based features to the TAN problem as they can theoretically be compared with known current profiles in an area of interest. More information as well as examples for water tracking and current profiling can be found [here](https://github.com/Field-Robotics-Lab/dave/wiki/DVL-Water-Tracking).


#### 1.3 Ultra-Short Baseline
Ultra-Short Baseline(USBL) is used to assist underwater navigation and positioning. It consists of one transceiver and one or multiple transponders (or beacons). Usually, the transceiver will be attached to either a stationary object or mobile central node, whereas the transponder(s) is attached to objects that are being tracked. For more information, see [this](https://www.youtube.com/watch?v=ZYTqp2thhZA&ab_channel=Sonardyne) video

   - Tutorials
     - [Ultra-Short Baseline sensor Wiki](https://github.com/Field-Robotics-Lab/dave/wiki/usbl_tutorial)

### 2. Positional Error Reset

#### 2.1 Utilizing GPS sensor
GPS has two main functions in a TAN system. For scenarios where GPS is available it can be used to continuously validate the results of a TAN system working separately from the GPS. In GPS-denied scenarios, the role of the GPS is relegated to situations where a vehicle may surface and determine its position. The vehicle can then calculate the positional error for the TAN system, reset its known location and orientation, and adjust its heading for subsequent GPS-denied actions. Details regarding the GPS plugin used within the Dave project can be found [here](http://wiki.ros.org/hector_gazebo_plugins)

### 3. Feature-based Navigation

#### 3.1 Utilizing Multibeam Echosounder for Forward-Looking Sonar (FLS) images
A Multibeam Echosounder (MBES; Multibeam Sonar) measurement can be used to obtain terrain datasets that correspond to multiple altimeter measurements at multiple locations. [Multibeam sonar plugin](https://github.com/Field-Robotics-Lab/nps_uw_multibeam_sonar) is designed as a separate plugin that can extend the capabilities of the Dave project. It requires the proprietary NVIDIA graphics card in order to utilize GPU parallel calculations. For the Blueview P900 sonar case, which has 512 separate beams, the refresh rate of the sonar image it can generate can go up to 10 Hz for a 10-meter distance range. These multiple separate beams measuring the distance of the terrain simultaneously make optimal sonar sensors for bathymetric terrain navigation. Unlike the Lidar or Radar sensors, the acoustic measurement includes strong correlations between beams as speckle noise making the final image much blurry. This is a critical physical phenomenon for real-world sonar sensors and the feature is included based on the physical model (the point scattering model) in the plugin.

[[/images/multibeam_sonar_summary.gif|alt=summary]]

   - Introduction of the MBES sensor plugin

     Previous sonar sensor plugins were based on image processing realms by translating each subpixel (point cloud) of the perceived image to resemble sonar sensors with or without sonar equations ([Detailed Review for the previous image-based methods](https://github.com/Field-Robotics-Lab/dave/wiki/image_sonar_description)). Here, we have developed a ray-based multibeam sonar plugin to consider the phase and reberveration physics of the acoustic signals providing raw sonar intensity-range data (the A-plot) using the point scattering model. Physical characteristics including time and angle ambiguities and speckle noise are considered. The time and angle ambiguity is a  function of the point spread function of the coherent imaging system (i.e.,  side lobes due to matched filtering and beamforming). Speckle is the granular appearance of an image that is due to many interfering scatterers that are smaller than the resolution limit of the imaging system.

   - Features
      - Physical sonar beam/ray calculation with the point scattering model
        - Generating intensity-range (A-plot) raw sonar data
        - Publishes the data with UW APL's sonar image msg format
      - NVIDIA CUDA core GPU parallelization
        - 10Hz refresh rate with 10m range
   - Tutorials
     - [Multibeam Forward-Looking Sonar Wiki](https://github.com/Field-Robotics-Lab/dave/wiki/Multibeam-Forward-Looking-Sonar)


#### 3.2 Utilizing DVL Sensor for Bathymery gradient estimation
High frequency seafloor features are generally not useful in TAN applications as they are subject to change in dynamic underwater environments. Seafloor gradients are more stable features that can be sampled using an AUV's DVL sensor. By comparing the collected seafloor gradients with those of a known bathymetry map, gradient features can be matched to improve navigation outside the DVL's traditional role of bottom-tracking for dead reckoning.
   - Tutorial
     - [DVL Seabed Gradient Visualization](https://github.com/Field-Robotics-Lab/dave/wiki/DVL-Seabed-Gradient)


### 4. Terrain Feature Set

#### 4.1 Incorporating Bathymetry Model
A bathymetry integration plugin included in the Dave project automatically spawns and removes bathymetry grids converted preliminarily from high-resolution NOAA bathymetry data.
- Features
   - Automatic spawn/remove with vehicle locations
   - Overlaps for mission continuity

- Generating bathymetry tiles to pipeline into the Gazebo world

   The bathymetry data need to be converted to be imported using the plugin. The converter to obtain bathymetry tiles from raw data downloaded from NOAA can be found at [Bathymetry converter](https://github.com/Field-Robotics-Lab/Bathymetry_Converter). Follow [this tutorial](https://github.com/Field-Robotics-Lab/dave/wiki/Bathymetry-Data-Conversion-for-Bathymetry-Plugin). Any bathymetry data format that can be read by the GDAL library can be processed.

- [Video tutorial link (2 min 50 sec)](https://youtu.be/jxifYkoMM3w?t=135)

[![Video tutorial link (2 min 50 sec)](https://img.youtube.com/vi/jxifYkoMM3w/0.jpg)](https://youtu.be/jxifYkoMM3w?t=135)


### 5. Evaluation
#### 5.1 Extracting vehicle position using Gazebo service call
Every model called in the Gazebo world generically includes Gazebo service call [`get_model_state`](http://docs.ros.org/en/jade/api/gazebo_msgs/html/srv/GetModelState.html) to obtain vehicle position and orientations. Tutorials on how to use service calls are at [ROS communication - Get Model State Example](http://gazebosim.org/tutorials/?tut=ros_comm#GetModelStateExample).

