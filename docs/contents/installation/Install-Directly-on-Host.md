---
layout: default
title: 404
nav_exclude: true
---

This tutorial will walk you through the setup required to make a host machine ready to build and run the Dave simulations. Note that:
* your host will need to satisfy the minimum [System Requirements](/dave/contents/installation/System-Requirements), and
* the steps below assume you are running **Ubuntu 20.04**.

## Install all dependencies
Upgrade to the latest packages:
```bash
sudo apt update
sudo apt full-upgrade
```

Install required tools:
```bash
sudo apt install -y build-essential cmake cppcheck curl git gnupg libeigen3-dev libgles2-mesa-dev lsb-release pkg-config protobuf-compiler python3-dbg python3-pip python3-venv qtbase5-dev ruby software-properties-common sudo wget
```

Install required ROS and Gazebo Packages
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update
DIST=noetic
GAZ=gazebo11
sudo apt install -y ${GAZ} lib${GAZ}-dev python3-catkin-tools python3-rosdep python3-rosinstall python3-rosinstall-generator python3-vcstool ros-${DIST}-gazebo-plugins ros-${DIST}-gazebo-ros ros-${DIST}-gazebo-ros-control ros-${DIST}-gazebo-ros-pkgs ros-${DIST}-effort-controllers ros-${DIST}-geographic-info ros-${DIST}-hector-gazebo-plugins ros-${DIST}-image-view ros-${DIST}-joint-state-controller ros-${DIST}-joint-state-publisher ros-${DIST}-joy ros-${DIST}-joy-teleop ros-${DIST}-kdl-parser-py ros-${DIST}-key-teleop ros-${DIST}-move-base ros-${DIST}-moveit-commander ros-${DIST}-moveit-planners ros-${DIST}-moveit-simple-controller-manager ros-${DIST}-moveit-ros-visualization ros-${DIST}-pcl-ros ros-${DIST}-robot-localization ros-${DIST}-robot-state-publisher ros-${DIST}-ros-base ros-${DIST}-ros-controllers ros-${DIST}-rqt ros-${DIST}-rqt-common-plugins ros-${DIST}-rqt-robot-plugins ros-${DIST}-rviz ros-${DIST}-teleop-tools ros-${DIST}-teleop-twist-joy ros-${DIST}-teleop-twist-keyboard ros-${DIST}-tf2-geometry-msgs ros-${DIST}-tf2-tools ros-${DIST}-urdfdom-py ros-${DIST}-velodyne-gazebo-plugins ros-${DIST}-velodyne-simulator ros-${DIST}-xacro
```
Note on specific package dependencies:
- The manipulator arm requires `ros-${DIST}-effort-controllers`
- `glider_hybrid_whoi` repository requires `ros-${DIST}-hector-gazebo-plugins`
- `nps_uw_multibeam_sonar` repository requires `ros-${DIST}-pcl-ros` and `ros-${DIST}-image-view` (see full dependencies on the [multibeam tutorial page](/dave/contents/Multibeam-Forward-Looking-Sonar))

Set up `.bashrc` for working with ROS Noetic:
```bash
source /opt/ros/noetic/setup.bash
```

### Verify Gazebo setup
Run Gazebo to verify that it starts:
```
gazebo
```
Verify that a supported version of gazebo is installed with
```
gazebo --version
```

which should yield something like...
```
Gazebo multi-robot simulator, version 11.5.1
```
## Next: [Get the Source Code](/dave/contents/installation/Clone-Dave-Repositories)

