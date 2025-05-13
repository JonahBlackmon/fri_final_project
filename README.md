# UR5 Greeting and Imitation

This repository contains the source code, data, and research paper for a project focused on enabling a UR5 robotic arm to recognize human gestures, perform social greetings, and imitate movements in real time. The system leverages ROS2, MoveIt 2, and custom perception and control scripts to build a foundational node system for manipulating the UR5.

---

## Project Summary

We developed a human-robot interaction framework in which a UR5 robot uses gesture recognition to:
- Identify social greeting gestures (e.g., wave, handshake motion)
- Respond appropriately through robot gestures
- Perform real-time imitation of user movements

This system demonstrates an integration of perception, planning, and execution in robotic manipulation for socially-aware behavior.

---

## Technologies Used

- **Robot Platform**: UR5 (Universal Robots)
- **Middleware**: ROS2 (Foxy)
- **Motion Planning**: MoveIt 2
- **Languages**: Python 3, C++
- **Tools**: OpenCV (gesture recognition), RViz, Gazebo

---

## Installation & Setup

### ROS2 Dependencies
Ensure that you have ROS2 Humble installed and configured. Then, install the required ROS2 dependencies:

```bash
sudo apt update
sudo apt install ros-humble-trajectory-msgs ros-humble-sensor-msgs ros-humble-std-msgs ros-humble-builtin-interfaces ros-humble-cv-bridge

### Python Dependencies
pip install opencv-python mediapipe numpy

### 1. Clone and Build
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/JonahBlackmon/ur5-greeting-imitation.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash

### Running the Node Network
#### Gesture Recognition and Greeting:
ros2 run recognition_node gesture_recognition_node
ros2 run ur5_wave wave_node
### Real-time Hand Imitation:
ros2 run mimic_hand mimic_node
ros2 run calculate_angle angle_node


