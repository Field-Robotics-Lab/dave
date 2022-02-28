---
layout: default
title: Notes
nav_exclude: true
---

**These notes were prepared in pre-version 1 and may no longer be relevant.**
* Platform versions: ROS Melodic, Ubuntu 18, Gazebo 9
* Bosch UUV Simulator, https://roscon.ros.org/2018/presentations/ROSCon2018_uuvsimulator.pdf
  * plume tracking
  * Gazebo plugins and ROS modules

# Working with uuv_simulator

* If installed from package for ros-kinetic, `roslaunch --files uuv_descriptions upload_rexrov.launch` shows:

      /opt/ros/melodic/share/uuv_assistants/launch/message_to_tf.launch
      /opt/ros/melodic/share/uuv_descriptions/launch/upload_rexrov_default.launch
      /opt/ros/melodic/share/uuv_descriptions/launch/upload_rexrov.launch

  Rather than installing and using this package, build it from source under workspace `uuv_ws`.

* When installed from our `uuv_ws` workspace, `roslaunch --files uuv_descriptions upload_rexrov.launch` shows:

      /home/bdallen/uuv_ws/src/uuv_simulator/uuv_descriptions/launch/upload_rexrov.launch
      /home/bdallen/uuv_ws/src/uuv_simulator/uuv_descriptions/launch/upload_rexrov_default.launch
      /home/bdallen/uuv_ws/src/uuv_simulator/uuv_assistants/launch/message_to_tf.launch

  and `roslaunch --files uuv_gazebo_worlds ocean_waves.launch` shows:

      /opt/ros/melodic/share/gazebo_ros/launch/empty_world.launch
      /home/bdallen/uuv_ws/src/uuv_simulator/uuv_gazebo_worlds/launch/ocean_waves.launch
      /home/bdallen/uuv_ws/src/uuv_simulator/uuv_assistants/launch/publish_world_ned_frame.launch

# Exploring uuv_simulator
In page [[uuv_simulator reference]] we launched the ocean waves world and spawned a UUV at coordinate(0,0,-20).  Here we examine the `.launch` files, `.world` files, and other code that makes this happen.  Later we explore additions to this to provide a target capability: Grasp a ceramic cup.

## `ocean_waves.launch`
Launch file `ocean_waves.launch` does this:

* Include Gazebo's `empty_world.launch`.
* Include `uuv_assistants` file `publish_world_ned_frame.launch`.
* Start node `publish_world_models` running `publish_world_models.py`.
* Optionally launch `uuv_assistants` file `set_simulation_timer.launch`.

## `upload_rexrov.launch`
On default, starts `upload_rexrov_default.launch` with initial x,y,z,roll,pitch,yaw values.


# `empty_world.launch`
This launch file, https://github.com/ros-simulation/gazebo_ros_pkgs/blob/jade-devel/gazebo_ros/launch/empty_world.launch, starts Gazebo, defining default parameters and opening `/usr/share/gazebo-9/empty.world`:

    <?xml version="1.0" ?>
    <sdf version="1.5">
      <world name="default">
        <!-- A global light source -->
        <include>
          <uri>model://sun</uri>
        </include>
        <!-- A ground plane -->
        <include>
          <uri>model://ground_plane</uri>
        </include>
      </world>
    </sdf>

which defines world "default" with light and ground.

# `ocean_waves.launch`
For launch file syntax, see http://wiki.ros.org/roslaunch.


# Sensors
File `~/uuv_ws/src/uuv_simulator/uuv_assistants/templates/robot_model/urdf/sensors.xacro.template` identifies example sensors: GPS, Pose 3D sensor, forward-looking sonar, DVL, RPT, Pressure, IMU, camera.  See `~/uuv_ws/src/uuv_simulator/uuv_sensor_plugins/uuv_sensor_ros_plugins/urdf/*.xacro` for more.

File `~/uuv_ws/src/uuv_simulator/uuv_descriptions/robots/rexrov_sonar.xacro` has `xacro:forward_multibeam_sonar_m450_130`.  It uses plugin name `uuv_plugin` filename `libuuv_underwater_object_ros_plugin.so` built from `~/uuv_ws/src/uuv_simulator/uuv_gazebo_plugins/uuv_gazebo_ros_plugins/src/UnderwaterObjectROSPlugin.cc`.  It connects services for modeling underwater properties.

File `~/uuv_ws/src/uuv_simulator/uuv_sensor_plugins` has plugins, for example `~/uuv_ws/src/uuv_simulator/uuv_sensor_plugins/uuv_sensor_ros_plugins/include/uuv_sensor_ros_plugins/gazebo_ros_image_sonar.hh`, `.cpp`.  From `CMakeLists.txt` this makes `image_sonar_ros_plugin.so`.  File `image_sonar_ros_plugin.so` is referenced in `~/uuv_ws/src/uuv_simulator/uuv_sensor_plugins/uuv_sensor_ros_plugins/urdf/sonar_snippets.xacro`.  File `sonar_snippets.xacro` is referenced in `~/uuv_ws/src/uuv_simulator/uuv_sensor_plugins/uuv_sensor_ros_plugins/urdf/sensor_snippets.xacro`.  File `sensor_snippets.xacro` is included from the following:

* `~/uuv_ws/src/uuv_simulator/uuv_assistants/templates/robot_model/urdf/base.xacro.template`
* `~/uuv_ws/src/uuv_simulator/uuv_descriptions/urdf/rexrov_base.xacro`
* `~/uuv_ws/src/uuv_simulator/uuv_sensor_plugins/uuv_sensor_ros_plugins/urdf/sensor_snippets.xacro`
* `~/uuv_ws/src/uuv_simulator/uuv_tutorials/uuv_tutorial_rov_model/urdf/rov_example_base.xacro`

File `rexrov_base.xacro` is included in several `rexrov_*` files, including `~/uuv_ws/src/uuv_simulator/uuv_descriptions/robots/rexrov_oberon7.xacro`, which is included in the `~/uuv_ws/src/uuv_simulator/uuv_descriptions/launch/upload_rexrov_oberon7.launch` file which we are using, meaning that the `image_sonar_ros_plugin` plugin is available in Gazebo.

# Move the UUV
A systematic approach to activating the UUV.
## Move from command line

* Launch ocean waves and UUV oberon7
* Find topics
* Move something via a topic

Launch ocean_waves

    roslaunch uuv_gazebo_worlds ocean_waves.launch

Using `rosrun rqt_graph rqt_graph` we see nodes `/publish_world_models`, `/gazebo`, `,world_ned_frame_publisher`, and `gazebo_gui`.  Using `rostopic list` we see some Gazebo topics and `/hydrodynamics/current_velocity`.

Launch rexrov_oberon7

    roslaunch uuv_descriptions upload_rexrov_oberon7.launch mode:=default x:=0 y:=0 z:=-20 namespace:=rexrov

In `rqt_graph`, clicking Refresh, we see nodes `/rexrov/ground_truth_to_tf_rexrov` and `/rexrov/robot_state_publisher`.  Topics between rexrov and Gazebo include `/rexrov/pose_gt`, `/tf`, `/tf_static`, `/rexrov/joint_states`.

Using `rostopic list` we see many `/rexrov/*` topics including various sensors and eight thrusters.

About the thrusters:  Thruster 0 is at `/rexrov/thrusters/0` and has these topics:

* `dynamic_state_efficiency`
* `input`
* `is_on`
* `thrust`
* `thrust_efficiency`
* `thrust_wrench`

See `~/uuv_ws/src/uuv_simulator/uuv_gazebo_plugins/uuv_gazebo_ros_plugins/src/ThrusterROSPlugin.cc` and `~/uuv_ws/src/uuv_simulator/uuv_gazebo_plugins/uuv_gazebo_plugins/src/ThrusterPlugin.cc`.  From `ThrusterPlugin.cc` `thrust` is of type `<msgs::Vector3d>`.  From `rostopic info /rexrov/thrusters/0/thrust`, it is of type `uuv_gazebo_ros_plugins_msgs/FloatStamped` and is published by Gazebo.  Topic `input` is subscribed by Gazebo and is of type `uuv_gazebo_ros_plugins_msgs/FloatStamped`.


## Move UUV using Joystick `js0`
Move the UUV by launching `joy_accel.launch`:

    roslaunch uuv_control_cascaded_pid joy_accel.launch model_name:=rexrov

We can specify the Joystick ID, default `joy_id:=0`.  This starts `thruster_manager.launch` and `uuv_teleop.launch`.  The teleop uses the two joystick controls for up/down/rotate/move while the thruster manager turns these commands into `thrust` commands for the eight thrusters.

## Move Arm
From `robot_config.yaml`, Oberon 7P manipulators have names: `azimuth`, `shoulder`, `elbow`, `roll`, `pitch`, `wrist`.  Arm setup is in `joint_effort_controllers.launch`.  This file is used in `uuv_manipulators/oberon7/oberon7_control/launch/jt_cartesian_controller.launch` and `uuv_manipulators/oberon7/oberon7_control/launch/joint_control.launch`.  These are used in `uuv_simulator/uuv_gazebo/launch/rexrov_demos/rexrov_oberon_demo.launch` and `uuv_simulator/uuv_gazebo/launch/rexrov_demos/rexrov_oberon_arms_demo.launch`.

In looking at `jt_cartesian_controller.launch` and `joint_control.launch`, we see that `joint_control.launch` has arm names, so try that.  See `joint_control.launch` for arm actuations, including gripper open/close and RB and LB deadman.  Try this:

    roslaunch oberon7_control joint_control.launch uuv_name:=rexrov

No, this fails: `effort_controllers/JointEffortController does not exist.`  Note: there was no `joy_id:=0` option, which seems odd.

# Sonar
See files with `sonar` in their name.  Working up from `uuv_simulator/uuv_sensor_plugins/uuv_sensor_ros_plugins/src/gazebo_ros_image_sonar.cpp` and `uuv_simulator/uuv_sensor_plugins/uuv_sensor_ros_plugins/include/uuv_sensor_ros_plugins/gazebo_ros_image_sonar.hh` we find `uuv_simulator/uuv_sensor_plugins/uuv_sensor_ros_plugins/CMakeLists.txt` which builds `image_sonar_ros_plugin` and others including rpt, imu, cpc, magnetometer, dvl, pressure, gps.

File `libimage_sonar_ros_plugin.so` is laded from `uuv_simulator/uuv_sensor_plugins/uuv_sensor_ros_plugins/urdf/sonar_snippets.xacro` which is loaded from `uuv_simulator/uuv_sensor_plugins/uuv_sensor_ros_plugins/urdf/sensor_snippets.xacro`.  Several .xacro files load this, including `uuv_simulator/uuv_descriptions/urdf/rexrov_base.xacro`, which should be loaded from `rexrov_oberon7.xacro`.

File `gazebo_ros_image_sonar.cpp` plugin `GazeboRosImageSonar` should provide these topics:

* point cloud `points` or from `pointCloudTopicName`, type `sensor_msgs::PointCloud2`
* depth image `depth/image_raw` or from `depthImageTopicName`, type `sensor_msgs::Image`
* normal image from depth image + `_normals`, type `sensor_msgs::Image`
* multibeam image from depth image + `_multibeam`, type `sensor_msgs::Image`
* sonar image from depth image + `_sonar`, type `sensor_msgs::Image`
* raw sonar image from depth image + `_raw_sonar`, type `sensor_msgs::Image`
* depth image camera info `depth/camera_info` or from `depthImageCameraInfoTopicName`

We might be interested in `depth/image_sonar`, emitted by calling `cv::Mat GazeboRosImageSonar::ConstructVisualScanImage(cv::Mat& raw_scan)`.  These use structures `depth_image` and its derivatives `normal_image`, `multibeam_image`, and `raw_scan`.  `depth_image` comes from `cv::Mat(height, width, CV_32FC1, (float*)_src)` where `CV_32FC1` is a 32-bit float 1-channel array of value `_src`.

Gazebo's `DepthCameraPlugin.hh` includes bindings for Gazebo events `OnNewDepthFrame`, `OnNewRGBPointCloud`, and `OnNewImageFrame`.


## Using `rqt_image_view`
This GUI offers a list of all compatible active "camera" topics.  Usage:

    rosrun rqt_image_view rqt_image_view

For example `/rexrov/rexrov/camera/camera_image`.  Unfortunately, `/rexrov/dvl_sonar` related topics have incompatible types.  Our dvl types are:

* `/rexrov/dvl` type `uuv_sensor_ros_plugins_msgs/DVL`
* `/rexrov/dvl_sonar0` type `sensor_msgs/Range`

# SyllogismRXS Sonar
In paper "A Computationally-Efficient 2D Imaging Sonar Model for Underwater Robotics Simulations in Gazebo", DeMarco produces Sonar images as follows:

1) Calculate point-cloud using angle of incidence, distance, and surface reflectivity.  The reflectivity of a surface is defined by `laser_retro` defined in the surface's SDF file, values 0 to 255.
1) Add Gaussian noise, transform to sonar image, add median blur, add salt-pepper noise.

File `imaging_sonar_sim.cpp` runs a ROS node that subscribes to a point cloud topic of type `sensor_msgs::PointCloud` and publishes an image of type `sensor_msgs::ImagePtr` which should just be a pointer to `sensor_msgs::Image`.