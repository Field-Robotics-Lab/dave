---
layout: default
title: 404
nav_exclude: true
---

Now that you've set up your development environment and obtained the source code, you can build the project by running:

```bash
cd ~/uuv_ws
catkin_make
```

When the build is finished, source your new `setup.bash`:
```bash
source ~/uuv_ws/devel/setup.bash
```
Optionally, you may wish to add `source ~/uuv_ws/devel/setup.bash` to your `.bashrc` file.

### Experimental

An alternative to `catkin_make` is to use the newer `catkin build`:
```
sudo apt-get update
sudo apt-get install python3-catkin-tools

# Optionally, you can configure to install the packages
catkin config --install

# Build
catkin build

# Source devel/setup.bash or install/setup.bash depending on whether you used the installation option
```

This is experimentally supported until all the CMake dependencies and installations in our packages have been tested.

## Test your Installation
Test that the installation is working by running one of the Dave demos
```
roslaunch dave_demo_launch dave_demo.launch
```
If the simulator does not close promptly with Ctrl-C, you can use the command `pkill gzclient && pkill gzserver` in another terminal window to force immediate shutdown.

![uuv_reference_view](../images/uuv_reference_view.png)

More details are discussed at [uuv_simulator_reference](/dave/contents/installation/uuv_simulator_reference).