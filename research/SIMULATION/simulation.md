# ROS Robot Modeling and Simulation with Gazebo

## Packages for Robot Modeling and Simulation
- URDF (Unified Robot Description Format): XML format for representing a robot model
  - urdf_parser_plugin: Parses URDF files into a robot model
  - urdfdom_headers: Headers for URDF parsers
  - collada_parser: Parses Collada files into a robot model
  - urdfdom: Populates data structures from URDF files
- joint_state_publisher: Reads the robot model description, finds all joints, and publishes joint values to all of the nonfixed joints
- joint_state_publisher_gui: GUI for joint_state_publisher. It allows you to drag the joints of the robot to see how the robot will move.
- kdl_parser: Parses robot descriptions in the Kinematics and Dynamics Library (KDL) format into a robot model
- robot_state_publisher: Uses the kinematic description of the robot to publish the state of each link in the robot model. It does this by listening to the joint states and then computing the forward kinematics of the robot and publishing the results as a tf transform and a message of type sensor_msgs/JointState.
- xacro: XML macro language. It allows you to construct shorter and more readable XML files by using macros that expand to larger XML expressions.


## URDF (Unified Robot Description Format)

URDF stands for Unified Robot Description Format, which is an XML file format used in ROS (Robot Operating System) to describe the kinematic and visual properties of a robot.

URDF files define the physical properties of the robot, such as its links, joints, sensors, and visual elements. This information is used by various ROS packages for tasks such as robot visualization, collision detection, motion planning, and control.

The URDF file describes the robot's kinematics using a tree structure of links and joints, where each link represents a rigid body and each joint represents a degree of freedom. URDF files can also include information about the robot's visual appearance, such as meshes and textures.

### URDF Elements
- link: Represents a single link of a robot. Using this tag, we can model a robot link and its propeties, including the size, shape, and color. It can also contain special dynamic properties, including intertia, visual, and collision.
- joint: Represents a joint between two links. Using this tag, we can model a robot joint and its properties, including the type, axis, and limits. It can also contain special dynamic properties, including dynamics, limits, safety_controller, calibration, and mimic. There are several types of joints, including fixed, continuous, revolute, prismatic, floating, and planar. A URDF is joined together by connecting the links with joints using a parent-child relationship.'
- robot: Represents a robot. Using this tag, we can model a robot and its properties, including the name, links, joints, transmissions, and plugins.
- gazebo: Represents a Gazebo element. Using this tag, we can model a Gazebo element and its properties, including the reference, material, and sensor.
- sensor: Represents a sensor. Using this tag, we can model a sensor and its properties, including the name, type, and parent.

### Xacro
Xacro is an XML macro language. It allows you to construct shorter and more readable XML files by using macros that expand to larger XML expressions. Xacro is used in ROS to simplify the creation of URDF files. It allows you to define macros for commonly used elements, such as links and joints, and then use those macros in your URDF files. This makes it easier to create and maintain URDF files.