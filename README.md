# Holo Battalion – Multi-Robot Warehouse Automation System

Developed as part of the **e-Yantra Robotics Competition (eYRC) 2025–26**, IIT Bombay.

## Overview

Holo Battalion is a ROS2-based multi-robot warehouse automation system developed to automate crate sorting and transportation inside a simulated warehouse environment.

The system consists of three holonomic mobile robots—**Glacio**, **Crystal**, and **Frostbite**—that collaboratively transport temperature-sensitive crates to their designated storage locations.

The project combines **robot perception, autonomous navigation, embedded systems, and robot control** to perform coordinated pick-and-place operations.

---

# Demo

▶️ Gazebo Simulation

https://www.youtube.com/watch?v=ffOcRU38SAU

▶️ Real Robot Demonstration

https://www.youtube.com/watch?v=DBNaWMGV1Pk

---

# Project Architecture

```
                 Overhead Camera
                        │
                        ▼
                 OpenCV Processing
                        │
           ArUco Detection + Homography
                        │
                        ▼
         Robot & Crate Pose Estimation
                        │
       ROS2 Topics (/bot_pose, /crate_pose)
                        │
                        ▼
               Task Allocation Logic
                        │
                        ▼
            PID Controller + Navigation
                        │
                        ▼
          Inverse Kinematics Calculation
                        │
                        ▼
              ROS2 MQTT Communication
                        │
                        ▼
                     ESP32
                        │
        ┌───────────────┼───────────────┐
        ▼               ▼               ▼
    MG995 Motors     MG90 Servo      Solenoid
                        │
                        ▼
                  Pick and Place
```

---

# System Workflow

1. An overhead camera captures the complete warehouse.

2. OpenCV detects ArUco markers placed on robots and crates.

3. Homography converts image coordinates into real-world coordinates.

4. Robot and crate poses are published using ROS2 topics.

5. A task allocation node assigns the nearest crate to each robot.

6. PID controllers generate smooth motion commands.

7. Inverse kinematics converts robot velocity into wheel velocities.

8. Commands are transmitted through MQTT to the ESP32.

9. ESP32 controls motors, servo motors, and solenoid to perform pick-and-place operations.

---

# Key Features

- Multi-robot warehouse automation
- ROS2-based communication
- Holonomic robot control
- Autonomous waypoint navigation
- PID-based motion control
- ArUco marker localization
- Homography-based coordinate transformation
- OpenCV-based color detection
- MQTT communication with ESP32
- Pick-and-place mechanism

---

# Technologies Used

## Robotics

- ROS2 Humble
- Gazebo
- URDF

## Programming

- Python
- Embedded C

## Computer Vision

- OpenCV
- ArUco Marker Detection
- Homography

## Embedded Systems

- ESP32
- MQTT
- PWM Control

## Hardware

- MG995 DC Motors
- MG90 Servo
- IR Sensor
- Solenoid
- Buck Converter

---

# Hardware Components

- ESP32
- 12V Li-Po Battery
- Buck Converter
- MG995 Motors
- MG90 Servo
- MOSFET Driver
- IR Sensor
- Solenoid

---

## Repository Structure

```text
Holo-Battalion/
│
├── hb_ws/
│   └── src/
│       └── eyrc-25-26-holo-battalion/
│
├── hb_ws2/
│   └── src/
│       └── eyrc-25-26-holo-battalion/
│
├── hardware_testing/
│   └── eyrc-25-26-holo-battalion/
│
├── esp32/
│
├── .gitignore
└── README.md
```

---

## Repository Organization

This repository follows the development workflow used during the **e-Yantra Robotics Competition (eYRC) 2025–26**.

The project was developed incrementally across multiple competition tasks, with each task maintained in a dedicated Git branch.

### Branches

| Branch | Description |
|---------|-------------|
| **main** | Initial boilerplate repository provided for project development. |
| **task1** | Initial ROS2 workspace (`hb_ws`) and Task 1 implementation. |
| **task2a** | Task 2A implementation in simulation. |
| **task4a** | Task 4A implementation in simulation. |
| **task4b** | Task 4B implementation in simulation. |
| **task5a** | Task 5A implementation in simulation. |
| **task5b** | Task 5B implementation in simulation. |
| **task6a** | Final simulation implementation. |
| **real_world_task** | Real-world implementation using `hb_ws2`, hardware integration, and ESP32 firmware. |

---

## Workspace Organization

### hb_ws

Contains the ROS2 simulation workspace developed during the competition tasks.

### hb_ws2

Contains the ROS2 workspace used for real-world robot deployment and hardware testing.

### hardware_testing

Contains hardware-specific development and testing files.

### esp32

Contains firmware for ESP32-based robot control.

This firmware is used only in the real-world implementation and was developed during the later stages of the competition (Task 4 onwards).

```

---

# Results

- Successfully completed autonomous crate sorting in simulation.

- Demonstrated coordinated operation of three holonomic robots.

- Successfully integrated perception, navigation, control, and embedded hardware.

- Achieved Level 2 in the e-Yantra Robotics Competition.

- Recognized as the Top Girls Team.

---

# Technical Challenges

- Accurate robot localization using overhead camera

- Homography calibration

- PID tuning for stable navigation

- Multi-robot coordination

- Hardware and ROS2 communication through MQTT

---

# Future Improvements

- Dynamic obstacle avoidance

- Advanced path planning

- SLAM integration

- Real-time warehouse deployment

- Autonomous charging station

---

# Project Resources

📄 Project Portfolio

drive.google.com/file/d/17czBFY-3adRa97wIfVcHAVCxyTsdHKzo/view

💻 GitHub Repository

https://github.com/nidhipal1027/Holo-Battalion

---

# Team

Team eYRC#1832

- Nidhi Pal
- Seeya Kokam
- Kashisa Padhy

---

Developed for the e-Yantra Robotics Competition (eYRC) 2025–26, IIT Bombay.
