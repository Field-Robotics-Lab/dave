---
layout: default
title: Installation
nav_order: 2
has_children: true
---

# System Setup
This guide explains how to build and run the Dave simulation environment. Make sure to review the [System Requirements](/dave/contents/installation/System-Requirements) before you begin.

## Step 1: Set up Your Development Environment
To build the Dave software, you need a development environment with the necessary dependencies installed (these include ROS, Gazebo and some utilities). There are two ways to do this:

### Option A: [Configure Your Host Machine](/dave/contents/installation/Install-Directly-on-Host)
Set up your host machine as your development environment.
* This involves installing all dependencies directly on the host itself.
* This is the simplest option provided you are willing to set up your machine to use the specific version of Ubuntu/ROS/Gazebo listed in the [System Requirements](/dave/contents/installation/System-Requirements).

### Option B: [Use a Docker Image](/dave/contents/installation/Docker-Development-Image)
Set up a Docker container with the necessary dependencies.
* This involves installing Docker, then installing dependencies into a Docker image.
* The image functions like a lightweight virtual machine that allows you to build and run Dave.
* This option is a little more complex conceptually, but has the advantage of leaving the configuration of your host machine (mostly) undisturbed.
* Use this option if you are using the same host for multiple development projects that make use of different software environments (for example, different ROS versions), or just prefer not to install packages on your host system.

## Step 2: [Get the Source Code](/dave/contents/installation/Clone-Dave-Repositories)
After completing either Option A or B above, follow this tutorial to clone the Dave repositories.

## Step 3: [Build the Dave Simulation Environment](/dave/contents/installation/Build-Dave-Environment)
Finally, build and run the Dave software itself.

## Experimental
As listed in the system requirements, Dave expects to be run on Ubuntu Linux, and we officially support Ubuntu 20.04/Noetic/Gazebo11. However, users who want to test experimental Windows support may be interested in the following tutorial:
* [Install on Windows using WSL2](/dave/contents/installation/Install-on-Windows-using-WSL2): Native-like methods to install on Windows 10 or 11