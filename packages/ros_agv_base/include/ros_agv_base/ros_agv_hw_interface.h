#ifndef ROSAGV_HW_INTERFACE_H
#define ROSAGV_HW_INTERFACE_H

// ROS
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/JointState.h>

// ROS Control
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

namespace rosagv_base {
    class RobotHWInterface : public hardware_interface::RobotHW {
        public:
            RobotHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model = NULL);

            // Initialize the hardware interface
            bool init();

            void read(const ros::Time& time, const ros::Duration& period);
            void write(const ros::Time& time, const ros::Duration& period);

            bool isReceivingMeasuredJointStates(const ros::Duration &timeout);
            void measuredJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

        protected:
            ros::NodeHandle nh_;

            std::array<std::string, 2> joint_names = {"left_wheel_joint", "right_wheel_joint"};

            hardware_interface::JointStateInterface joint_state_interface_;
            hardware_interface::VelocityJointInterface velocity_joint_interface_;

            sensor_msgs::JointState joint_state_msg_;

            ros::Publisher wheel_cmd_pub_;
            ros::Subscriber joint_state_sub_;
            ros::ServiceServer srv_start_;
            ros::ServiceServer srv_stop_;

            std::vector <std::string> joint_names_;
            urdf::Model *urdf_model_;

            double reg_joint_velocity_commands[2];
            double reg_joint_positions[2];
            double reg_joint_velocities[2];
            double reg_joint_efforts[2];

            double wheel_radius;
            double wheel_separation;
            double max_velocity;
            double pwm_limit; 

            void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg);
    };
}

#endif