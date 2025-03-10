# **ros2_marker_based_nav**

## **Project Overview**
This project is part of the **ENPM809Y** course at the **University of Maryland, College Park**. The goal of the project is to develop an **Autonomous Robot Navigation system** using **ROS2**. The robot navigates a maze using **ArUco markers**, detects objects using a **logical camera**, and transforms frames for pose estimation.

![Robot Image](rwa3_image.JPG)


---

## **Table of Contents**
- [Project Overview](#project-overview)
- [Features](#features)
- [Installation](#installation)
  - [Step 1: Clone the Repository](#step-1-clone-the-repository)
  - [Step 2: Install Dependencies](#step-2-install-dependencies)
  - [Step 3: Build the Package](#step-3-build-the-package)
- [Running the Project](#running-the-project)
  - [Step 1: Launch the ROS2 Simulation](#step-1-launch-the-ros2-simulation)
  - [Step 2: Start the Subscriber Node](#step-2-start-the-subscriber-node)
  - [Step 3: Start Robot Navigation](#step-3-start-robot-navigation)
- [Project Structure](#project-structure)


---

## **Features**
✔ Uses **ArUco markers** for navigation decisions.  
✔ Implements a **ROS2 subscriber** for marker detection.  
✔ Reads **waypoint parameters** from `params.yaml`.  
✔ Uses **TF2 transformations** to align world and camera frames.  
✔ Detects objects in the environment using a **logical camera**.  
✔ Commands **TurtleBot3** to move based on detected markers.  

---

## **Installation**
### **Prerequisites**
Ensure you have the following dependencies installed:
- **Ubuntu 22.04** (or later)
- **ROS2 Humble**
- **Gazebo** (for simulation)
- **Nav2 (Navigation Stack 2)**
- **ArUco marker detection**
- **TF2 for transformations**
- **Colcon for package compilation**

### **Step 1: Clone the Repository**
```bash
cd ~
mkdir -p ~/rwa3_ws/src
cd ~/rwa3_ws/src
git clone <your-repository-url> group16
```
### **Step2: Install Dependencies**
```bash
cd ~/rwa3_ws
rosdep install --from-paths src --ignore-src -r -y

```

### **Step 3: Build the Package**
```bash
cd ~/rwa3_ws
colcon build --packages-select group16
source install/setup.bash

```
## **Running the Project**
### **Step 1: Launch the ROS2 Simulation**
```bash
ros2 launch turtlebot3_gazebo maze.launch.py
```
### **Step 2: Start the Subscriber Node**
```bash
ros2 run group16 code
```

### **Step 3: Start Robot Navigation**
```bash
ros2 launch group16 listen.launch.py
```

## **Project Structure**
```
group16/
├── config/
│   ├── params.yaml              # Contains ArUco marker parameters
│
├── include/group16/
│   ├── header.hpp               # Header file for node definitions
│
├── launch/
│   ├── listen.launch.py         # ROS2 launch file for the navigation node
│
├── src/
│   ├── code.cpp                 # Main code for the publisher node
│   ├── listen.cpp               # Node that listens for marker detections
│
├── CMakeLists.txt               # ROS2 CMake build configuration
├── package.xml                  # Package dependencies and metadata
├── README.md                    # Documentation

```
## **Additional Notes**

If you encounter any issues, check that all dependencies are installed correctly using:
```
rosdep install --from-paths src --ignore-src -r -y
```
Ensure your workspace is properly sourced:
```
source install/setup.bash
```
Contributions and pull requests are welcome!

