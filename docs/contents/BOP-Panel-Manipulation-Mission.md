---
layout: default
title: BOP Panel Manipulation
nav_exclude: true
---

# Overview

You will need a joystick to perform the actual mission.

This launch file starts Gazebo with the RexROV spawned in front of a bop panel. The purpose of this is to demonstrate the capabilities and functionalities of the manipulator. The robot is in velocity control mode, meaning that the robot will hover in position when there is no input provided to the controller.

Run the following command in a sourced terminal
```
roslaunch dave_demo_launch dave_valve_manipulation_demo.launch
```

This launch includes the following:
* Launches the uuv_dave_ocean_waves world
* Spawns the RexROV with the oberon7 manipulator and the joint controller node for the manipulator
* Joystick velocity controller
* Spawns the BOP panel
* Launch rviz and rqt_image_view

After running the launch command, this is what the Gazebo environment should look like:

[[images/bop_panel_manip_overview.png|alt=overview]]

The task is to use the front camera's feed to close/open valves by turning the knob dials on the BOP panel using the oberon7 manipulator arm.

[[images/bop_panel_rqt_image_view.png|alt=rqt_image_view]]

## Velocity Control mode

The `joy_velocity.launch` that was included in this launch file starts a `velocity_control` node that enables the UUV to hold its position when there are no command given to the controller.  The [same controls](https://github.com/Field-Robotics-Lab/dave/wiki/Logitech-F310-Gamepad-Mapping) still apply for controlling the robot and the UUV arm.

[This video](https://vimeo.com/419861065) talks more about the velocity control teleop mode.

## Full mission

A full mission associated with setup can be seen in the following video:

[![bop panel manipulation mission](https://img.youtube.com/vi/vKMR8-7WRF4/0.jpg)](https://www.youtube.com/watch?v=vKMR8-7WRF4&feature=youtu.be)

[This video](https://vimeo.com/420142173) gives further explanations about the BOP panel manipulation mission.
