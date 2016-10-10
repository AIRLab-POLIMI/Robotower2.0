// This program subscribes to wiimote and publishes the interested button state.
#include<ros/ros.h>

int main (int argc, char** argv){
	// Initialize the ROS system and become a node .
	ros::init(argc, argv, "imu_sensor");
	ros::NodeHandle nh;

	// Let ROS take over .
	ros::spin();
}
