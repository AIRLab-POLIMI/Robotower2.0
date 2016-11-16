/* This node publishes the data received from the imu MPU-6050 module.
author: Ewerton Lopes

*/

#include<ros/ros.h>
#include<ros/console.h>
#include <time.h>
#include<robogame_imu_sensor/imu_state.h>
#include<robogame_imu_sensor/imu_test.h>

// Publisher
ros::Publisher pub;

void handle_msg(const robogame_imu_sensor::imu_state& msg){
    robogame_imu_sensor::imu_test newmsg;
    newmsg.header.stamp = ros::Time::now();
	newmsg.linacc_x = msg.linear_acc.x;
    pub.publish(newmsg);
}

int main (int argc, char** argv){
	// Initialize the ROS system and become a node .
	ros::init(argc, argv, "robogame_imu_tester");
	ros::NodeHandle nh;

	// Create a subscriber object.
	ros::Subscriber sub = nh.subscribe("robogame/imu_state", 1000, &handle_msg);
	pub = nh.advertise<robogame_imu_sensor::imu_test>("robogame/imu_test",1000);
	
	// Let ROS take over.
	ros::spin();
}
