---
layout: default
title: Home
nav_order: 1
description: "Ros-vehicle Docs"
permalink: /
---

# Ros-vehicle Docs
{: .fs-9 }

The entire project is documented here including how to get it up and running!
{: .fs-6 .fw-300 }

[Get started now](#getting-started){: .btn .btn-primary .fs-5 .mb-4 .mb-md-0 .mr-2 } [View it on GitHub](https://github.com/mubarizahmed/ros-vehicle){: .btn .fs-5 .mb-4 .mb-md-0 }

---

## Getting started

### Installing ROS
  ROS Noetic has been used for this project. Detailed installation instructions can be found in the official [ROS Wiki](http://wiki.ros.org/noetic/Installation). The following is a summary for installation on ubuntu:

  1. Set up sources

      ```sh
      sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
      ```
  2. Set up keys

      ```sh
      sudo apt install curl # if you haven't already installed curl

      curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
      ```

  3. Install ROS

      ```sh
      sudo apt update

      sudo apt install ros-noetic-ros-base
      ```

  4. Setting up the environment
      ```sh
      echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
      source ~/.bashrc
      ```

  5. Set up dependancies
      ```sh
      sudo apt-get install python3-rosinstall python3-rosinstall-generator python3-wstool build-essential ros-noetic-roslint
      ```

  6. Install and initialize rosdep
      ```sh
      sudo apt install python3-rosdep
      sudo rosdep init
      rosdep update
      ```

### Initializing the workspace
  A workspace has to be initialized where all the ROS packages are kept.

  1. Make and navigate to the workspace directory
      ```sh
      mkdir -p ~/catkin_ws/src
      cd catkin_ws/src
      ```
  2. Initialize workspace
      ```sh
      catkin_init_workspace
      ```
  3. Build the workspace
      ```sh
      cd ~/catkin_ws
      catkin_make
      ```
  3. Add the workspace path to shell
      ```sh
      echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
      source ~/.bashrc
      ```
### Dependencies
  The following packages are required additionally:

  1. **rosserial_python** | [Docs](http://wiki.ros.org/rosserial_python)
      ```sh
      sudo apt install ros-noetic-rosserial-python
      ```
  2. **teleop_twist_gamepad** | [Docs](https://github.com/Reinbert/teleop_twist_gamepad)
      ```sh
      cd ~/catkin_ws/src
      git clone https://github.com/Reinbert/teleop_twist_gamepad
      cd ..
      catkin_make
      ```
