# Holo Battalion

## Project Overview

Holo Battalion is an advanced multi-robot warehouse automation system inspired by swarm intelligence, developed by Team eYRC#1832 as part of the e-Yantra Robotics Competition (eYRC) 2025–26 at IIT Bombay.

The system consists of three holonomic robots—Glacio, Crystal, and Frostbite—that collaboratively perform efficient crate sorting and placement operations within a constrained warehouse environment. Each robot is equipped with an articulated arm and an omnidirectional drive, enabling precise movement in all directions, including lateral motion and in-place rotation for handling crates in confined spaces.

---

## Problem Statement

The objective is to design a coordinated multi-robot system capable of autonomously performing crate picking, identification, transportation, and placement while ensuring collision-free navigation and optimal space utilization.

---

## Crate Classification

Crates are identified using color-based classification:

* Blue
* Red
* Green

Each crate is assigned to a designated storage zone based on its color.

---

## System Features

* Multi-robot coordination
* Swarm intelligence-based decision making
* Holonomic drive for omnidirectional mobility
* Autonomous navigation and path planning
* Collision avoidance
* Dynamic path re-planning
* Space-efficient crate arrangement

---

## Swarm Intelligence

The system incorporates swarm intelligence principles to enable coordinated and adaptive behavior among robots, including dynamic task allocation, collision-free navigation, and efficient collaboration in shared environments.

---

## Technologies Used

* ROS 2 (Robot Operating System)  
* Gazebo Simulation  
* URDF (Unified Robot Description Format)  
* Python  
* Linux (Ubuntu 22.04)  
* Computer Vision and Image Processing  
* Coordinate Transformations  
* Holonomic Drive Kinematics  
* Path Planning and Navigation Algorithms  
* Multi-Robot Coordination and Swarm Intelligence  

---

## Working Mechanism

1. Robots detect crates using sensors or vision systems
2. Crates are classified based on color
3. Optimal paths are planned for transportation
4. Robots pick and transport crates to designated zones
5. Crates are placed efficiently within storage areas
6. The process continues with coordinated multi-robot operation

---

## Objective

To achieve efficient, reliable, and collision-free warehouse automation, demonstrating practical applications in logistics, industrial automation, and intelligent warehousing systems.

---

## Team Members (Team ID: eYRC#1832)

* Nidhi Pal
* Seeya Kokam
* Kashisa Padhy

---

## Competition

Developed as part of the e-Yantra Robotics Competition (eYRC) 2025–26 conducted by IIT Bombay.

---

## Future Scope

* Deployment in real-world warehouse environments
* Integration with intelligent optimization systems
* Scalability to larger multi-robot fleets
* Enhanced perception and object tracking capabilities

---

## License

This project is intended for academic and competition purposes.
