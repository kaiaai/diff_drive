#  TortoiseBot Simulation – ROS 2 Jazzy

##  Project Overview

This project demonstrates how to **simulate TortoiseBot in ROS 2 Jazzy** using Gazebo and RViz2.
The setup includes:

* `tortoisebot_description` → URDF and visualization in RViz2.
* `tortoisebot_gazebo` → spawn the robot in an empty Gazebo world.
* Velocity command interface (`/cmd_vel`) for robot control.
* Teleoperation using `teleop_twist_keyboard`.
* Path Tracker Node → follows a predefined path by publishing velocity commands.

---


##  Features

* Load and visualize **TortoiseBot** in RViz2.
* Spawn TortoiseBot in **Gazebo empty world**.
* Control robot using keyboard teleop.
* Custom **Path Tracker Node** for autonomous movement.

---


##  System Requirements

* **OS:** Ubuntu 24.04 LTS
* **ROS 2:** Jazzy Jalisco
* **Gazebo:** Fortress or Harmonic

##  Installation & Setup

### 1️ Install ROS 2 Jazzy

Follow the [official instructions](https://docs.ros.org/en/jazzy/Installation.html):

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install ros-jazzy-desktop ros-jazzy-gazebo-ros-pkgs
```

### 2️ Setup a ROS 2 Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 3️ Clone This Repository

```bash
git clone https://github.com/<sivapriya083>/<differential_drive_robot_control>.git
```

### 4️ Install Dependencies

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 5️ Build the Workspace

```bash
colcon build
source install/setup.bash
```

---


##  How to Run the Simulation


### Launch RViz with TortoiseBot

```bash
ros2 launch tortoisebot_description display.launch.py
```
If the robot does'nt load automatically in rviz2,click the RobotModel dropdown arrow in the RViz2 pannel and type /robot_description infront of the description topic. 

![TortoiseBot in RViz2](https://github.com/Sivapriya083/differential_drive_robot_control/blob/d36d88c3b8e573951dc676682f3dc8520f1bc50f/robot%20in%20rviz.png)


### Spawn TortoiseBot in Gazebo

```bash
ros2 launch tortoisebot_gazebo empty_world.launch.py
```
![TortoiseBot in gz](https://github.com/Sivapriya083/differential_drive_robot_control/blob/main/robot%20in%20gz.png?raw=true)


### Run Teleop Node

In new terminal, start the teleoperation:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Movement:

i → move forward

, → move backward

j → turn left

l → turn right

u → move forward-left (diagonal)

o → move forward-right

m → move backward-left

. → move backward-right

## Stop:

k → stop

## Speed Adjustments:

q → increase linear speed

z → decrease linear speed

w → increase angular speed

x → decrease angular speed

## Exit:

CTRL+C → quit the node


### Run Path Tracker Node

```bash
ros2 run tortoisebot_gazebo path_tracker
```
Add aruco marker  and path from the RViz2 panel and select the Topic: /visualization_marker_array and Topic: /robot_path for path visualization.


![TortoiseBot in gz](https://github.com/Sivapriya083/differential_drive_robot_control/blob/main/robot%20simulation.png?raw=true)

---

##  Project Structure

```
tortoisebot_description/
│── urdf/           # URDF files for robot model
│── launch/         # RViz launch files

tortoisebot_gazebo/
│── worlds/         # Gazebo world files
│── launch/         # Gazebo launch files
│── src/            # Path tracker node
```

##  References

* [teleop\_twist\_keyboard](https://github.com/ros2/teleop_twist_keyboard)
* [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)

---







































