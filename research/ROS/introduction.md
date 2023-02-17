# Introduction to Robot Operating System (ROS)

The Robot Operating System (ROS) is a open-source set of software libraries and tools that help you build robotic applications. ROS provides drivers and state-of-the-art algorithms along with powerful developer tools. There are packages for computing trajectory, conduct SLAM algorithms, remote control, and much more.  

ROS is not a true operating system since it runs on top of Ubuntu, a linux based distribution. It is more like a middleware that provides a core with communication tools, hardware abstractions, and a set of plug & play libraries. There are many reasons to use ROS as a framework for robotic applications, and some of them are as follows:

- ROS comes with packages that implement state-of-the-art algorithms, including packages such as Simultaneous Localization and Mapping (SLAM) and Adaptive Monte Carlo Localization (AMCL) for integrating autonomous navigation for mobile robots
- ROS provides many tools for debugging, visualizing, and simulating robotic systems. Open Source Tools such as RViz, rqt_graph, and Gazebo are powerful for designing, implementing, and testing robotic systems. 
- ROS takes care of concurrent resource handling of hardware resources. We can write a single-threaded piece of code and create nodes for concurrency. 

## ROS Filesystem

- **Packages**: ROS packages are a core element of ROS. They contain ROS nodes, libraries, and configuration files that are organized together as a single unit
- **Package Manifest**: Contains information about the package
- **Metapackages**: Refers to one or more packages that are loosely group. 

