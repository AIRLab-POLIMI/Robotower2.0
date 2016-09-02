// This program subscribes to wiimote and publishes the interested button state.
#include<ros/ros.h>
#include<wiimote/State.h>
#include<robogame_rosarduino/State.h>

int previousButtonState = 0;
// Create a publisher object.
ros::Publisher pub;
ros::Subscriber sub;

// A callback function . Executed each time a wiimote message arrives
void stateMessageReceived(const wiimote::State& msg){
	int newButtonState = msg.buttons[5];
	if (previousButtonState != newButtonState){
		previousButtonState = newButtonState;
		// Create and fill in the message.
		robogame_rosarduino::State buttonmsg;
		// Publish the message.
		buttonmsg.state = newButtonState;
		pub.publish(buttonmsg);
	}
}

int main (int argc, char** argv){
	// Initialize the ROS system and become a node .
	ros::init(argc, argv, "ros_arduino_wiimote_listener");
	ros::NodeHandle nh;

	// Create a subscriber object.
	sub = nh.subscribe("wiimote/state", 1000, &stateMessageReceived);
	pub = nh.advertise<robogame_rosarduino::State>("wiimote/fit_arduino_state",1000);

	ROS_INFO_STREAM("Listening to target wiimote button event...");

	// Let ROS take over .
	ros::spin();
}
