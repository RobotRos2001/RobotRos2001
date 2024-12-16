<p align="center"><strong>tita_rl_simulation</strong></p>
<p align="center"><a href="https://github.com/${YOUR_GIT_REPOSITORY}/blob/main/LICENSE"><img alt="License" src="https://img.shields.io/badge/License-Apache%202.0-orange"/></a>
<img alt="language" src="https://img.shields.io/badge/language-c++-red"/>
<img alt="platform" src="https://img.shields.io/badge/platform-linux-l"/>
</p>
<p align="center">
    语言：<a href="./docs/docs_en/README_EN.md"><strong>English</strong></a> / <strong>中文</strong>
</p>

## Description

This project is a TITA robot control project based on ROS2, using the DRL algorithm for control.

## Prerequisites

- **Operating System**: Ubuntu 22.04
- **ROS 2**: Humble
- **Gazebo**: classic

## Dependencies

```bash
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers

# build webots
sudo apt install ros-humble-webots-ros2
# build gazebo
sudo apt install ros-humble-gazebo-ros2-control
sudo apt install ros-humble-gazebo-ros
```

## Build Package

```bash
mkdir tita_ws/src && cd tita_ws/src
git clone https://github.com/RobotRos2001/TITA_GAZEBO_DRL.git
colcon build
source install/setup.bash
ros2 launch TITA_DRL tita_drl.launch.py
ros2 launch sim_bringup sim_bringup.launch.py
```
