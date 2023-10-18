# Differential drive wheeled mobile robot
This repository contains packages for Autonomous Carrier Robot.

[![N|Solid](https://avatars.githubusercontent.com/u/3979232?s=280&v=4)](https://www.ros.org/)

## Hardware components:
1. NUC with ubuntu 20
2. Hoverboard driver (my reference firmware code and hardware: https://github.com/EFeru/hoverboard-firmware-hack-FOC)
3. Intel RealSense 455 (firmware version: D400_Series_FW_5_13_0_55)
4. LiDAR A1

# Requirement:
1. ROS2 Foxy
- realsense_ros: https://github.com/IntelRealSense/realsense-ros/tree/ros2-legacy)
- Nav2 stack: https://github.com/ros-planning/navigation2/tree/foxy-devel
- ros2_control: https://github.com/ros-controls/ros2_control/tree/foxy
5. YOLOv8

## Features

- Human following Robot
- Waypoint robot with A* Algorithm and Dynamic Window Approach algorithm

## Installation
```sh
$ cd ~/robotta_ws/src
$ git clone https://github.com/AlfretzNehemia/robotta_ws.git
$ pip3 install -r robotta_vision/requirements.txt
$ cd ~/robotta_ws
$ rosdep install --from-paths src --ignore-src -r -y
$ colcon build
```

## Find a bug?
If you found an issue or would like to submit an improvement to this project, please submit an issue using the issues tab above. If you would like to submit a PR with a fix, reference the issue you created!
