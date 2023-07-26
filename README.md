# ROS Automated Guided Vehicle Research Project

A undergraduate research project for developing an autonomous differential drive mobile robot using ROS and the ROS navigation stack. This project is currently in the development phase and I recently finished a working prototype. See the research folder for notes about the project as I am researching and learning differential drive robots.  

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
- [x] Setup Navigation Stack with AMCL (Adaptive Monte Carlo Localization) and move_base
- [ ] Create Flask app to remotely control and monitor the robot

<img src="./media/phase_two_cad.png"  width="600" >

## Latest Progress 7/25/23
Generated a SLAM map of my basement using the graph-based SLAM algorithm from the slam_toolbox package. The map was generated using the RPLidar A1M8 sensor with a resolution of 0.3. Had some troubles getting asynchronous online SLAM to generate the map properly over my local network. As a result, I switched to Synchronous online SLAM and that seemed to resolve the issue. 

Here is a [Video demo](https://www.youtube.com/watch?v=7yjPUBrIlA8) of using slam_toolbox to generate a map of my basement with Synchronous Online SLAM. 

<img src="./media/basement_0_map.png" width="600">



## Future Endevaours 
- OpenCV Object Recognition with YOLOv7 (You Only Look Once) Model
- Flask Webserver for Teleoperation and Configuration of Robot
