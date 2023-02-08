# ROS Automated Guided Vehicle

A research project for developing an autonomous mobile robot using the ROS navigation stack. This project is still in the research phase and is not expected to be compeleted anytime in the near-future. 

## Phase One Goals
- Design a chassis for a simplified version of the robot
- Integrate L298N Motor Controllers with TT 3-6V DC Motors
- Develop Firmware for MEGA2560
- Integrate with HC-05 to communicate with the vehicle via Bluetooth 2.0
- Integrate HC-SR04 for basic obstacle avoidance and roaming
- Develop simple control app in Kotlin
- Test/calibrate DC encoders for Phase Two Integration
- Test/calibrate IMU for Phase Two Integration

<img src="./media/phase_one_cad.png"  width="600" >

## Phase Two Goals
- Integrate High-Performance SBC with ROS
- Setup Communication between embedded controller and ROS
- Create initial ROS structure with Teleop of Robot
- Integrate Bosch IMU and Wheel Encoder using localization package and Extended Kalman Filter (EKF)
- Aquire SLAM Data with RPLidar A1M8 using ROS libraries and packages

## Phase Three Goals
- Finalize Localization 
- Autonomously Navigate to Destination
- Hazard Detection