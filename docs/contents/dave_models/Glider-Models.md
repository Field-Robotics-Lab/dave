---
layout: default
title: Dave Glider Models
parent: Underwater Vehicle Models
grand_parent: Dave Models
nav_order: 3
---

# Static examples to illustrate visual models

## Underwater gliders

The following launch file starts the ocean world and adds each of the three gliders: Slocum, hybrid and wave.

```
roslaunch dave_demo_launch dave_gliders_visual_demo.launch paused:=true
```
To see each glider, expand the `Models` on the left side panel of the Gazebo window and right click the glider name and click `Move To`

![image](https://user-images.githubusercontent.com/7955120/139202251-cf9d9000-3b84-4502-bd56-912ec8eff33e.png)

This command launches an example ocean floor and loads three gliders into the simulation world in the Gazebo. It is meant to launch with a `paused` flag since the hydrodynamics of the wave glider is not developed yet. It's mainly to demonstrate the visual of the three gliders.

If you want to try the newly developed physics engine of the glider dynamics, please visit https://github.com/Field-Robotics-Lab/glider_hybrid_whoi

### Slocum glider

![slocum_gazebo](../images/slocum_gazebo.png)

![slocum_rviz](../images/slocum_rviz.png)


### WHOI hybrid glider

![hybrid_glider](../images/hybrid_glider.png)

![hybrid_glider2](../images/hybrid_glider2.png)

![hybrid_glider_frames](../images/hybrid_glider_frames.png)


## Wave glider

![wave_glider](../images/wave_glider.png)


