# differential_drive_robot_control
This project demonstrates how to simulate TortoiseBot in Gazebo and control it using teleop_twist_keyboard.
The setup includes:

tortoisebot_description package → defines URDF and visualization in RViz.

tortoisebot_gazebo package → launches TortoiseBot inside an empty Gazebo world.

Velocity command interface (/cmd_vel) to move the robot.

Keyboard teleoperation for manual control.

Click [here](https://youtu.be/LdBMpnNu1Q4) for the simulation video.

## Features

Visualize TortoiseBot URDF in RViz2.

Spawn TortoiseBot in Gazebo with an empty world.

Manual control using teleop_twist_keyboard (/cmd_vel).

Path Tracker Node → follows a predefined path by publishing velocity commands

System Requirements

OS: Ubuntu 24.04 LTS (recommended)

ROS 2: Jazzy Jalisco

Gazebo: Fortress / Harmonic

Python: ≥ 3.10

C++: ≥ C++17

## Prerequisites & Installation
1. Install ROS 2 Jazzy

   Follow the official setup guide:
   
  
  ```bash sudo apt update && sudo apt upgrade -y
  ```bash sudo apt install ros-jazzy-desktop ros-jazzy-gazebo-ros-pkgs
