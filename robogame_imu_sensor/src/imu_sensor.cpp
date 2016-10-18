// This program subscribes to wiimote and publishes the interested button state.
#include<ros/ros.h>
#include<ros/console.h>
#include <time.h>
#include<robogame_imu_sensor/imu_state.h>
#include<robogame_imu_sensor/imu_test.h>

// Create a publisher object for button state.
ros::Publisher pub;

// A callback function . Executed each time a wiimote message arrives
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
	pub = nh.advertise<robogame_imu_sensor::imu_test>("robogame/imu_test",1000);				// advertise LED and Buzzer state

	// Let ROS take over .
	ros::spin();
}
