#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

int main(int argc, char **argv) {
    // Initialize the ROS Node
    ros::init(argc, argv, "rosagv_hw_interface");
    ros::NodeHandle nh;

    // Create an instance of your robot so that this instance can be passed to the controller manager.
    // This is where you set the hardware interface type, e.g. rosagv_base::RosAgvBaseHW
    //rosagv_base::RosAgvBase robot(nh);

    // Setup a separate thread that will be used to handle ROS callbacks to avoid blocking the realtime loop
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Setup control loop
    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(50.0); // 50Hz update rate
    rate.sleep();

    while(ros::ok()) {
        
    }

}