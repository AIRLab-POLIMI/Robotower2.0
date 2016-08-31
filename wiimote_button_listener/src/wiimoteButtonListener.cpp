// This program subscribes to wiimote and publishes the interested button state.
#include<ros/ros.h>
#include<wiimote/State.h>
#include<wiimote_button_listener/wii_target_button_state.h>

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
		wiimote_button_listener::wii_target_button_state buttonmsg;
		// Publish the message.
		buttonmsg.state = newButtonState;
		pub.publish(buttonmsg);
	}
}

int main (int argc, char** argv){
	// Initialize the ROS system and become a node .
	ros::init(argc, argv, "wiimote_listener");
	ros::NodeHandle nh;

	// Create a subscriber object.
	sub = nh.subscribe("wiimote/state", 1000, &stateMessageReceived);
	pub = nh.advertise<wiimote_button_listener::wii_target_button_state>("wiimote/target_button_state",1000);

	ROS_INFO_STREAM("Listening to target wiimote button event...");

	// Let ROS take over .
	ros::spin();
}
