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
Got it ✅ Azar — here’s a **complete README.md** you can drop directly into your repo. It’s polished, structured, and portfolio-ready:

---

# 🐢 TortoiseBot Simulation – ROS 2 Jazzy

## 📌 Project Overview

This project demonstrates how to **simulate TortoiseBot in ROS 2 Jazzy** using Gazebo and RViz2.
The setup includes:

* `tortoisebot_description` → URDF and visualization in RViz2.
* `tortoisebot_gazebo` → spawn the robot in an empty Gazebo world.
* Velocity command interface (`/cmd_vel`) for robot control.
* **Teleoperation** using `teleop_twist_keyboard`.
* **Path Tracker Node** → follows a predefined path by publishing velocity commands.

---

## ✨ Features

* Load and visualize **TortoiseBot** in RViz2.
* Spawn TortoiseBot in **Gazebo empty world**.
* Control robot using keyboard teleop.
* Custom **Path Tracker Node** for autonomous movement.

---

## 💻 System Requirements

* **OS:** Ubuntu 24.04 LTS
* **ROS 2:** Jazzy Jalisco
* **Gazebo:** Fortress or Harmonic
* **Python:** ≥ 3.10
* **C++:** ≥ C++17
* **Hardware:**

  * CPU: Dual-core i5 / Ryzen 3 or higher
  * RAM: ≥ 8 GB
  * GPU: Recommended for smoother Gazebo + RViz2 rendering

---

## 📦 Installation & Setup

### 1️⃣ Install ROS 2 Jazzy

Follow the [official instructions](https://docs.ros.org/en/jazzy/Installation.html):

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install ros-jazzy-desktop ros-jazzy-gazebo-ros-pkgs
```

### 2️⃣ Setup a ROS 2 Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 3️⃣ Clone This Repository

```bash
git clone https://github.com/<your-username>/<your-repo-name>.git
```

### 4️⃣ Install Dependencies

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 5️⃣ Build the Workspace

```bash
colcon build
source install/setup.bash
```

---

## ▶️ How to Run the Simulation

### Launch RViz with TortoiseBot

```bash
ros2 launch tortoisebot_description display.launch.py
```

### Spawn TortoiseBot in Gazebo

```bash
ros2 launch tortoisebot_gazebo empty_world.launch.py
```

### Run Teleop Node

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Run Path Tracker Node

```bash
ros2 run tortoisebot_gazebo path_tracker
```

---

## 📂 Project Structure

```
tortoisebot_description/
│── urdf/           # URDF files for robot model
│── launch/         # RViz launch files

tortoisebot_gazebo/
│── worlds/         # Gazebo world files
│── launch/         # Gazebo launch files
│── src/            # Path tracker node
```

---

## 🧩 Example: Path Tracker Node (Python)

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class PathTracker(Node):
    def __init__(self):
        super().__init__('path_tracker')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.move_straight)

    def move_straight(self):
        msg = Twist()
        msg.linear.x = 0.2  # move forward
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)
        self.get_logger().info("Moving forward...")

def main(args=None):
    rclpy.init(args=args)
    node = PathTracker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

---

## 📷 Demo

👉 Add a screenshot or video of your simulation here. Example:
![TortoiseBot in Gazebo](demo/tortoisebot_gazebo.png)

---

## 🔗 References

* [teleop\_twist\_keyboard](https://github.com/ros2/teleop_twist_keyboard)
* [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)

---

✅ With this README, anyone can install, run, and understand your project — and recruiters see the **extra Path Tracker node** as your unique contribution.

---

Do you also want me to add a **badges section** (ROS2 version, build status, license, etc.) at the top of the README to make it look even more professional?





































