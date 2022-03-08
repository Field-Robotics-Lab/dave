---
layout: default
title: System Requirements
parent: Installation
---

# System Requirements for Running dave
## Hardware

* A modern multi-core CPU, e.g. Intel Core i5
* 8 Gb of RAM
* A discrete Graphics Card, e.g. Nvidia GTX 650
    * The environment can be run without a GPU, but the Gazebo simulation will run much faster (should run in real-time) with access to a GPU. Without a GPU the simulation is likely to run slower than real-time.

## Software
 - Recommended
   * Ubuntu Desktop 20.04 Focal (64-bit)

 - Legacy mode
   * Ubuntu Desktop 18.04 Bionic (64-bit)
   * Gazebo 9.11.0+ (<9.11.0 is not sufficient)
   * ROS Melodic

### Nvidia Driver
Our tutorials assume you have an Nvidia graphics card configured to use the proprietary driver.
* There are many online guides for configuring the Nvidia driver correctly on Ubuntu.
* Here's [one example with graphical and command-line options](https://github.com/Field-Robotics-Lab/dave/wiki/linuxbabe.com/ubuntu/install-nvidia-driver-ubuntu-18-04).

### Docker
Running Docker is optional. If you choose to use the container-based installation instructions, the following are required:
* Docker v19.03 or higher ([installation instructions](https://docs.docker.com/engine/install/ubuntu/))
    * Make sure to also complete the Linux post-install instructions linked at the bottom of the page.
* nvidia-container-toolkit ([installation instructions](https://github.com/NVIDIA/nvidia-docker))

## Peripherals
We also recommend a gamepad for testing the UUV and its arm and sensor devices. In the examples we use a Logitech F310 ([Walmart](https://www.walmart.com/ip/Logitech-F310-GamePad/16419686)).