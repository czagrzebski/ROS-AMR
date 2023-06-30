# ROS Control

## Overview
ros_control is a software package in the Robot Operating System (ROS) framework that provides a standardized interface for controlling robot hawrdware. It offers a set of tools and libraries that enable the development of robot controllers and the integration of different robot hardware. It also provides a standardized interface for interfacing with robot hardware from different manufacturers. The main goal of ros_control is to provide a unified interface for controlling various robot joints and actuutators, regardless of their specific hardware implementation. It abstracts the underlying hardware details and provides a common interface for interacting with robot control loops, such as position, velocity, and effort control. This allows for greater flexibility and modularity in robot control systems, making it easier to develop and integrate different robot components. 

## Robot Hardware Abstraction
- Resource: actuators, joints, sensors, motors
- Interface: Set of similar resources
- Robot: Set of interfaces

## Arduino Mega 2560
- Runs low-level PID control loop
- Outputs the measured velocity of the motor and the number of encoder ticks