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
- [ ] Install Ubuntu and ROS Noetic on a SBC (Orange Pi 5)
- [ ] Create a ROS Package for the AGV
- [ ] Create URDF (Unified Robot Description Format) and xacro files
- [ ] Setup Simulation Environment with Gazebo 
- [ ] Create Program for Embedded Microcontroller for Motor Control and IMU Sensor Data Collection
- [ ] Publish IMU Data and Measured Wheel Velocities to ROS from Microcontroller
- [ ] Setup ROS Control 
- [ ] Setup Hardware Interface to handle communication between microcontroller and ROS (using ROSSerial)
- [ ] Setup and Test diff_drive_controller. Tune PID values for wheel velocity control
- [ ] Create a teleop package for manual control of the robot using keyboard or joystick
- [ ] Setup RPLidar A1M8 Sensor Interface
- [ ] Create package for SLAM (Simultaneous Localization and Mapping) with slam_toolbox
- [ ] Setup Navigation Stack with AMCL (Adaptive Monte Carlo Localization) and move_base


<img src="./media/phase_two_cad.png"  width="600" >

## Future Endevaours 
- OpenCV Object Recognition with YOLOv7 (You Only Look Once) Model
- Flask Webserver for Teleoperation and Configuration of Robot
