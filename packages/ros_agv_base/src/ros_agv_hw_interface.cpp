#include <ros_agv_base/ros_agv_hw_interface.h>

namespace rosagv_base
{
    RobotHWInterface::RobotHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model) : nh_(nh), urdf_model_(urdf_model) {
        // Initialize the hardware interface
        init();

        // Create a publisher for the wheel commands
        wheel_cmd_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("wheel_velocity_cmd", 10);

        // Create a subscriber for the joint states
        joint_state_sub_ = nh_.subscribe("joint_states", 10, &RobotHWInterface::measuredJointStateCallback, this);

    }

    bool RobotHWInterface::init() {
        ROS_INFO("Initializing ROS AGV Hardware Interface...");

        // Create joint state interface for all joints
        for (unsigned int i = 0; i < joint_names.size(); i++)
        {
            ROS_INFO_STREAM("Registering joint " << joint_names[i] << " in the joint state interface");
            
            // Create a read-only joint state handle for each joint and register them with the joint state interface
            hardware_interface::JointStateHandle joint_state_handle(joint_names[i], &reg_joint_positions[i], &reg_joint_velocities[i], &reg_joint_efforts[i]);
            joint_state_interface_.registerHandle(joint_state_handle);

            ROS_INFO_STREAM("Registering joint " << joint_names[i] << " in the velocity joint interface");
            // Create a read and write velocity joint handle for each joint and register them with the velocity joint interface
            hardware_interface::JointHandle joint_handle(joint_state_handle, &reg_joint_velocity_commands[i]);
            velocity_joint_interface_.registerHandle(joint_handle);

            // Initialize Registers
            reg_joint_positions[i] = 0.0;
            reg_joint_velocities[i] = 0.0;
            reg_joint_efforts[i] = 0.0;
            reg_joint_velocity_commands[i] = 0.0;
            
        }

        // Initialize joint state message
        joint_state_msg_.name.resize(joint_names.size());
        joint_state_msg_.position.resize(joint_names.size());
        joint_state_msg_.velocity.resize(joint_names.size());
        joint_state_msg_.effort.resize(joint_names.size());

        ROS_INFO("Registering joint state interface with ROS Control");
        registerInterface(&joint_state_interface_);

        ROS_INFO("Registering velocity joint interface with ROS Control");
        registerInterface(&velocity_joint_interface_);

        ROS_INFO("ROS AGV Hardware Interface Initialized");
        return true;
    }

    void RobotHWInterface::read(const ros::Time& time, const ros::Duration& period) {
        //ROS_INFO("ROS AGV Hardware Interface: Reading joint state data from the robot");
        // Read the joint states from the robot
        for (std::size_t i = 0; i < joint_names.size(); i++) {
            // Read the joint states from the robot
            reg_joint_positions[i] = joint_state_msg_.position[i];
            reg_joint_velocities[i] = joint_state_msg_.velocity[i];
            reg_joint_efforts[i] = joint_state_msg_.effort[i];
        }
    }

    void RobotHWInterface::write(const ros::Time& time, const ros::Duration& period) {
        //ROS_INFO("ROS AGV Hardware Interface: Writing velocity data to the robot");
        // Publish the wheel commands
        geometry_msgs::Vector3Stamped wheel_cmd_msg;
        wheel_cmd_msg.header.stamp = ros::Time::now();
        wheel_cmd_msg.vector.x = reg_joint_velocity_commands[0];
        wheel_cmd_msg.vector.y = reg_joint_velocity_commands[1];
        ROS_INFO_STREAM("Left Motor Velocity Command: " << reg_joint_velocity_commands[0]);
        wheel_cmd_msg.vector.z = 0.0;
        wheel_cmd_pub_.publish(wheel_cmd_msg);
    }

    void RobotHWInterface::measuredJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        //ROS_INFO("ROS AGV Hardware Interface: Received Joint States");
        // Update the joint state message using pointer aliasing
        joint_state_msg_ = *msg;
    }
}