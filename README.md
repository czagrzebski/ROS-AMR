# ROS Automated Guided Vehicle Research Project

A research project for developing an autonomous differential drive mobile robot using ROS and the ROS navigation stack. This project is still in the research phase and is not expected to be completed anytime in the near-future. See the research folder for notes about the project as I am researching and learning differential drive robots.  

## Version 1 Goals [Complete]
- [x] Design a chassis for a simplified version of the robot
- [x] Integrate L298N Motor Controllers with TT 3-6V DC Motors
- [x] Develop Firmware for MEGA2560
- [x] Test/calibrate DC encoders for Phase Two Integration
- [x] Test/calibrate IMU for Phase Two Integration

<img src="./media/phase_one_cad.png"  width="600" >

## Version 2 Goals [In Progress]
- [x] Design a CAD model for a differential drive AGV with a 3D printed chassis
- [x] Install Ubuntu and ROS Noetic on a SBC (Orange Pi 5)
- [x] Create a ROS Package for the AGV
- [x] Create URDF (Unified Robot Description Format) and xacro files
- [x] Setup Simulation Environment with Gazebo 
- [x] Create Program for Embedded Microcontroller for Motor Control 
- [x] Publish Measured Wheel Velocities to ROS from Microcontroller
- [x] Setup ROS Control 
- [x] Setup Hardware Interface to handle communication between microcontroller and ROS (using ROSSerial)
- [x] Setup and Test diff_drive_controller. Tune PID values for wheel velocity control
- [x] Create a teleop package for manual control of the robot using keyboard or joystick
- [x] Setup RPLidar A1M8 Sensor Interface
- [x] Create package for SLAM (Simultaneous Localization and Mapping) with slam_toolbox
- [x] Generate a map of the environment using SLAM
- [ ] Integrate Robot Localization Package with IMU and Odometry
- [ ] Setup Navigation Stack with AMCL (Adaptive Monte Carlo Localization) and move_base


<img src="./media/phase_two_cad.png"  width="600" >

## Latest Progress 7/16/23
Generated a SLAM map of my basement using the slam_toolbox package. The map was generated using the RPLidar A1M8 sensor. Had some troubles getting SLAM to work properly, but was fixed by flipping the laser frame 180 degrees on the Unified Robot Description Format (URDF) file.

Here is a [Video demo](https://www.youtube.com/watch?v=7yjPUBrIlA8) of using slam_toolbox to generate a map of my basement with Synchronous Online SLAM. 

<img src="./media/basement_0_map.png" width="600">



## Future Endevaours 
- OpenCV Object Recognition with YOLOv7 (You Only Look Once) Model
- Flask Webserver for Teleoperation and Configuration of Robot
