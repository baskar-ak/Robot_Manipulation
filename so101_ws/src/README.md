## Overview

This repository contains the required packages for the SO101 perception-driven motion planning.

The workspace includes:

- SO101 robot description
- Bringup launch files
- MoveIt configuration
- Behavior Tree template

## Prerequisites

- Ubuntu (22.04 or newer) 
- ROS2 (Humble or newer)  installed
- Isaac Sim installed and configured
- MoveIt2 compatible with ROS2 

## SO101 Robot Description
- `git clone https://github.com/TheRobotStudio/SO-ARM100`
- Inside Simulation > SO101, copy `assets` folder, `so101_new_calib.urdf` and `so101_new_calib.xml` files into `~/so101_ws/src`
- Then `ros2 pkg create --build-type ament_python so101_description`
- Inside `so101_description` folder paste `assets` folder, `so101_new_calib.urdf` and `so101_new_calib.xml` files inside `urdf` folder
- Modify `package.xml` and `setup.py`. URDF is ready!

## MoveIt2 Configuration
- bash > `ros2 run moveit_setup_assistant moveit_setup_assistant` (this launches MoveIt setup wizard)
- Follow this video to setup (https://www.youtube.com/watch?v=gLMvNKducy8&list=PLU_rF1cv2oRneZp6fsJ2U2Gsn2jY5F8ve&index=17)
- Once you configured everything, create a folder called `so101_moveit_config` inside `~/so101_ws/src`
- `so101_moveit_config` package should contain `config` and `launch` folders
- `colcon build` > `source install/setup.bash` > `ros2 launch so101_moveit_config demo.launch.py` (this will launch rviz and moveit)
MoveIt ready!

## Isaac Sim
- Load the URDF manually or from Isaac Assets > search for "so101"
- Create > Action graph (taught in the above video)
- Hit Play, while Plan & Execute in MoveIt (you should see the arm move in sync in Isaac sim and MoveIt)
