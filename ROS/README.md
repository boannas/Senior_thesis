# ROS2 Projects for SIM2REAL Motor Control and Sensor Fusion  

This repository contains two integrated ROS 2 systems developed for the **FRA502 Advanced Robotic Control & Simulation** and usage for **FRA501 Physical AI** used to validate and compare a **Sim2Real pipeline** for transferring reinforcement learning (RL) policies from simulation to real hardware. The overall objective is to establish a reliable **motor control module** and a **1-leg sensing module** that together support closed-loop control of a Faulhaber motor and a 1-degree-of-freedom robotic leg.

---

## Project Overview

### 1. Faulhaber Motor Control (STM32G474RE + micro-ROS)

This subsystem implements a **motor controller** for a Faulhaber DC motor using micro-ROS on an STM32G474RE.  
The controller is based on the **MIT position–velocity–torque control law**  

**Capabilities**
- Executes an MIT-style controller for:
  - Position tracking  
  - Velocity tracking  
  - Torque control  
- Publishes real-time motor states via ROS 2 topics at **100 Hz**:
  - Position  
  - Velocity  
  - Torque  
  - Voltage command  
- Provides a drop-in interface for **RL policy integration**:
  - RL policy receives reference signals  
  - Policy outputs an action (voltage command)  
  - System verifies SIM2REAL tracking performance

This module is intended for validating whether an RL policy trained in simulation can track target trajectories on the real motor.

---

### 2. 1-Leg Sensor Fusion System (IMU + Encoder + Kalman Filter)

This subsystem performs **state estimation** for a 1-degree-of-freedom robotic leg using IMU and encoder data.

**Features**
- Runs on **micro-ROS (STM32G474RE)**
- Uses a **Kalman Filter** to fuse IMU and encoder measurements
- Outputs stable, noise-reduced estimates for RL observation:
  - Linear position (x)  
  - Linear velocity (ẋ)  
  - Estimated IMU accelerometer bias   
- Designed as the **observation pipeline** for RL training and real-world deployment

The fusion system provides high-quality proprioceptive signals that help reduce the Sim2Real discrepancy.

---

## System Purpose within the Physical AI Subject

Together, these two modules form the foundation of a **Sim2Real experimental platform**:

1. **Motor Controller**  
   Executes commands and enforces real-world actuator dynamics.

2. **Sensor Fusion Node**  
   Supplies reliable real-time observations to an RL agent operating on real hardware.

This architecture enables systematic comparison between:
- Simulation-based RL behavior  
- Real-world motor and sensor responses  
- Traditional controllers (MIT) vs RL policies  

---



**ROS 2 Version:**  
- Developed and tested on **ROS 2 Humble (Ubuntu 22.04)**

---

## Repository Structure
```
ROSxxxx/
├── motor_controller.md # Faulhaber + MIT controller + micro-ROS firmware
├── sensor_fusion.md # IMU + encoder Kalman filter + micro-ROS
└── README.md # Overview
```

## Installation & ROS package Setup
1. Clone Repository 
```bash
cd ~
git clone xxxxxx 
```

2. Build and source ROS2 workspace
``` bash
# Before building, update all file paths to match your environment

cd xxxxx 
colcon build && source install/setup.bash
```
Each subsystem includes its own detailed setup guide and usage:

- `motor_controller.md`  
- `sensor_fusion.md`  



---

## Author  
Natthanicha T.  
Field Robotics (FIBO), King Mongkut’s University of Technology Thonburi


