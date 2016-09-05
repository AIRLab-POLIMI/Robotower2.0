// This program subscribes to wiimote and publishes the interested button state.
#include<ros/ros.h>
#include<wiimote/State.h>
#include<robogame_rosarduino/State.h>

#include<sensor_msgs/JoyFeedbackArray.h>
#include<sensor_msgs/JoyFeedback.h>

#include<time.h>

const int MAXLED = 4;
const int ON = 1;
const int OFF = 0;
int previousButtonState = 0;
int rumble = OFF;
bool AtStart = false;
int bullets[4] = {ON,ON,ON,ON};
int numBullets = 4;
int pressCounter;
std::time_t rumbleStart,rumbleEnd;

// Create a publisher object for button state.
ros::Publisher BUTTONpub;
// Create a LED publisher object.
ros::Publisher LEDRumble_pub;
ros::Subscriber sub;

// checks if the wiimote has all LEDs on.
bool isAllOn(const boost::array<unsigned char,4>  LEDsState){
	int count=0;
	for(int i=0;i<MAXLED;i++){
		count += LEDsState[i];
	}
	return (count == MAXLED);
}

void updateBullets(){
	for(int i=0;i<MAXLED;i++){
		if(i < numBullets){
			bullets[i] = ON;
		}else{
			bullets[i] = OFF;
		}
	}
}

// Receives a blank vector and fill it with the bullets data.
void setLEDs(sensor_msgs::JoyFeedbackArray& LEDstates){
	for(int ledid=0;ledid<MAXLED;ledid++){
		sensor_msgs::JoyFeedback led;
		led.type = sensor_msgs::JoyFeedback::TYPE_LED;
		led.id = ledid;
		led.intensity = bullets[ledid];
		LEDstates.array.push_back(led);
	}
}

// A callback function . Executed each time a wiimote message arrives
void stateMessageReceived(const wiimote::State& msg){

	//setting init
	if(!AtStart){
		if (isAllOn(msg.LEDs)){
			AtStart = true;
			numBullets = 4;
			pressCounter = 0;
		}else{
			sensor_msgs::JoyFeedbackArray msg;
			setLEDs(msg);
			// Publish the message .
			LEDRumble_pub.publish(msg);
		}
	}else{
		int newButtonState = msg.buttons[5];
		if (previousButtonState != newButtonState){
			previousButtonState = newButtonState;
			if ((newButtonState == ON) && (numBullets > 0)){
				ROS_INFO_STREAM("Button pressed");
				if(!((numBullets -1) < 0)) numBullets--;
				ROS_INFO_STREAM("Bullets left: " << numBullets);
				updateBullets();
				sensor_msgs::JoyFeedbackArray msg;
				setLEDs(msg);
				LEDRumble_pub.publish(msg);					// publish new wiimote LED state
				robogame_rosarduino::State buttonmsg;	// Create and fill in the message for arduino feedback (buzzer).
				buttonmsg.state = newButtonState;
				BUTTONpub.publish(buttonmsg);			// Publish the message.
			}else if ((newButtonState == ON) && (numBullets == 0)){
				rumble = ON;
				sensor_msgs::JoyFeedbackArray msg;
				sensor_msgs::JoyFeedback rumble;
				rumble.type = sensor_msgs::JoyFeedback::TYPE_RUMBLE;
				rumble.id = 0;
				rumble.intensity = 1;
				msg.array.push_back(rumble);
				LEDRumble_pub.publish(msg);
			}else if ((newButtonState == OFF) && (rumble == ON)){
				rumble = OFF;
				sensor_msgs::JoyFeedbackArray msg;
				sensor_msgs::JoyFeedback rumble;
				rumble.type = sensor_msgs::JoyFeedback::TYPE_RUMBLE;
				rumble.id = 0;
				rumble.intensity = 0;
				msg.array.push_back(rumble);
				LEDRumble_pub.publish(msg);
			}
		}
	}
}

int main (int argc, char** argv){
	// Initialize the ROS system and become a node .
	ros::init(argc, argv, "ros_arduino_wiimote_listener");
	ros::NodeHandle nh;

	// Create a subscriber object.
	sub = nh.subscribe("wiimote/state", 1000, &stateMessageReceived);
	BUTTONpub = nh.advertise<robogame_rosarduino::State>("wiimote/fit_arduino_state",1000);		// advertise button state for arduino listener
	LEDRumble_pub = nh.advertise<sensor_msgs::JoyFeedbackArray>("/joy/set_feedback",1000);		// advertise LED and Rumble state

	ROS_INFO_STREAM("Listening to wiimote button event...");
	ROS_INFO_STREAM("Obtained LED control...");

	// Let ROS take over .
	ros::spin();
}
