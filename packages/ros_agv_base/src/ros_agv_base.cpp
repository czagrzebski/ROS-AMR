#include <ros/ros.h>
#include <ros_agv_base/ros_agv_hw_interface.h>
#include <controller_manager/controller_manager.h>

int main(int argc, char **argv) {
    // Initialize the ROS Node
    ros::init(argc, argv, "rosagv_hw_interface");
    ros::NodeHandle nh;

    // Create an instance of your robot so that this instance can be passed to the controller manager.
    // This is where you set the hardware interface type, e.g. rosagv_base::RosAgvBaseHW
    rosagv_base::RobotHWInterface robot(nh);

    // Create a controller manager instance to handle resource access/allocation
    controller_manager::ControllerManager cm(&robot, nh);

    // Setup a separate thread that will be used to handle ROS callbacks to avoid blocking the realtime loop
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Setup control loop
    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(50.0); // 50Hz update rate
    rate.sleep();

    while(ros::ok()) {
        const ros::Time time = ros::Time::now();
        const ros::Duration period = time - prev_time;
        prev_time = time;

        // Execute control loop (read state into registers)
        robot.read(time, period);

        // Call update method to controller manager to update the controllers
        cm.update(time, period);

        // Execute control loop (write commands from registers)
        robot.write(time, period);

        // Sleep to maintain loop rate
        rate.sleep();
    }

    return 0;

}