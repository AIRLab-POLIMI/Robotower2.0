// This program subscribes to wiimote and publishes the interested button state.
#include<ros/ros.h>
#include<sensor_msgs/JoyFeedbackArray.h>
#include<sensor_msgs/JoyFeedback.h>

const int ON = 1;
const int OFF = 0;

int main (int argc, char** argv){
	// Initialize the ROS system and become a node .
	ros::init(argc, argv, "wiimote_LED_CONTROLER");
	ros::NodeHandle nh;

	// Create a publisher object.
	ros::Publisher pub = nh.advertise<sensor_msgs::JoyFeedbackArray>("/joy/set_feedback",1000);

	ROS_INFO_STREAM("Publishing LED events at 2Hz...");

	// Loop at 2Hz until the node i s shut down.
	ros::Rate rate(2);
	int count= 0;
	bool state = false;
	int ledid;
	while(ros::ok()){
		// Create and fill  in the message . The other four;
		// fields, which are ignored by turtlesim, default to 0.

		ledid = count%4;
		sensor_msgs::JoyFeedbackArray msg;
		sensor_msgs::JoyFeedback led1;
		led1.type = sensor_msgs::JoyFeedback::TYPE_LED;
		led1.id = ledid;

		if (ledid == 0) state = !state;
		led1.intensity = state;

		msg.array.push_back(led1);
		
		count++;

		// Publish the message .
		pub.publish(msg);

		// Wait until it's time for another iteration.
		rate.sleep();
	}
}
