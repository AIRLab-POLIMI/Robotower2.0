// This program subscribes to wiimote and publishes the interested button state.
#include<ros/ros.h>
#include<robogame_arduino/interState.h>
#include<robogame_game_manager/gameState.h>

bool LEDChanged = false;
bool beepChanged = false;

// Create a publisher object for button state.
ros::Publisher pub;

// A callback function . Executed each time a wiimote message arrives
void stateMessageReceived(const robogame_game_manager::gameState& msg){
    robogame_arduino::interState ardmsg;
    ardmsg.LEDColorindex = msg.LEDColorindex;
    ardmsg.beep = msg.beep;
    ardmsg.isPlayerLost = msg.isPlayerLost;
    ardmsg.isPlayerTooClose = msg.isPlayerTooClose;
    pub.publish(ardmsg);
}

int main (int argc, char** argv){
	// Initialize the ROS system and become a node .
	ros::init(argc, argv, "robogame_ros_arduino");
	ros::NodeHandle nh;

	// Create a subscriber object.
	ros::Subscriber sub = nh.subscribe("robogame/gameState", 1000, &stateMessageReceived);
	pub = nh.advertise<robogame_arduino::interState>("robogame/iteraction_state",1000);				// advertise LED and Buzzer state

    ROS_INFO_STREAM("ROS-ARDUINO: Listening to game state event...");
	
	// Let ROS take over .
	ros::spin();
}
