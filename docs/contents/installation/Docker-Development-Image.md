---
layout: default
title: 404
nav_exclude: true
---

This guide explains how to set up a Docker-based development environment for building and developing Dave repository and related software.

## Step 1: [Install Dependencies](https://github.com/Field-Robotics-Lab/dockwater/wiki/Install-Dependencies)
Follow these steps to install required utilities on your host system.

## Step 2: Build and run
Once you've installed the dependencies, you can build your Docker development environment with the following commands:
```
git clone https://github.com/Field-Robotics-Lab/dockwater.git
cd dockwater
./build.bash noetic
./run.bash dockwater:noetic
```
If the above is successful, you should end up with a command prompt opened into the Docker container. Your user information will be the same and your home directory will be mounted and accessible within the container.

### OPTIONAL: GPU Multibeam sonar
```diff
- DO NOT INCLUDE THIS if you are not using multibeam sonar.
- It require CUDA Library and NVIDIA driver along with the NVIDIA graphics card that supports CUDA feature.
```
For instructions and plugin details : [Multibeam Forward-Looking Sonar](/dave/contents/Multibeam-Forward-Looking-Sonar)
```bash
git clone -b cuda https://github.com/Field-Robotics-Lab/dockwater.git
cd dockwater
./build.bash noetic
./run.bash noetic:latest
```

### Notes
* The `build.bash` and `run.bash` scripts may take a few minutes the first time they run.
* Code for the Docker development images is hosted in the [dockwater repository](https://github.com/Field-Robotics-Lab/dockwater). See the [repository wiki](https://github.com/Field-Robotics-Lab/dockwater/wiki) for more information about this project and supported use cases.

## Next: [Get the Source Code](/dave/contents/installation/Clone-Dave-Repositories)