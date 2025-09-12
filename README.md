# F1TENTH ADAS Algorithms

This repository contains the algorithms developed as part of my **Bachelor’s Thesis** at the **University of Alcalá (UAH)**, within the framework of the [F1TENTH project](https://roboracer.ai/).  
The work focuses on the **implementation of Advanced Driver Assistance Systems (ADAS)** for an autonomous 1/10 scale racecar, both in **simulation** and in the **physical platform**.



## Project Overview

The goal of this project is to design, implement, and test **reactive navigation algorithms** for autonomous driving.  
The repository is divided into two main branches:

- **`f1tenth_gym`** → nodes for the [f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros) **simulator**.  
- **`f1tenth_ws`** → ROS2 workspace with nodes for the **physical car** running on a Jetson platform.

This structure allows algorithms to be tested first in a controlled simulation environment and then deployed on the real F1TENTH car.



## Implemented Algorithms

- **Safety Node**  
  Subscribes to LiDAR (`/scan`) and odometry (`/ego_racecar/odom`) topics.  
  Computes the **Time to Collision (TTC)** and stops the car if the threshold is exceeded.

- **Wall Following**  
  Keeps the vehicle parallel to a wall using LiDAR readings.  
  A reactive navigation algorithm that adjusts steering to maintain a set distance.

- **Follow the Gap**  
  Selects the **largest gap** in the LiDAR scan and steers through it.  
  Useful for navigating environments with obstacles.




## Tech Stack

- **ROS2** (Foxy/Humble)  
- **C++** (nodes implementation)  
- **Python** (auxiliary scripts)  
- **Docker** (for simulator)  
- **f1tenth_gym_ros** (simulation environment)  
- **NVIDIA Jetson** (onboard computer for the real car)  



## Usage

Clone the repository:

```bash
git clone https://github.com/emaarnolfo/TFG-F1Tenth.git
cd TFG-F1Tenth

# For simulation
git checkout f1tenth_gym

# For real car
git checkout f1tenth_ws
```


Each branch contains its own setup and nodes.
