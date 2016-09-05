// This program subscribes to wiimote and publishes the interested button state.
#include<ros/ros.h>
#include<wiimote/State.h>
#include<robogame_wiimote_listener/State.h>

#include<sensor_msgs/JoyFeedbackArray.h>
#include<sensor_msgs/JoyFeedback.h>

#include<time.h>

const int MAXLED = 4;									// Maximum LED number
const int ON = 1;
const int OFF = 0;
int prevShootBtState = OFF;
int prevRecharBtState = OFF;
int rumble = OFF;
int bullets[4] = {ON,ON,ON,ON};
int numBullets = 4;									// four initial bullets available at game start
bool AtStart = false;

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

// Update LED state based on the number of bullets
void updateBullets(){
	for(int i=0;i<MAXLED;i++){
		if(i < numBullets){
			bullets[i] = ON;
		}else{
			bullets[i] = OFF;
		}
	}
}

// Update Rumble state
sensor_msgs::JoyFeedbackArray updateRumble(float state){
	sensor_msgs::JoyFeedbackArray msg;
	sensor_msgs::JoyFeedback rumble;
	rumble.type = sensor_msgs::JoyFeedback::TYPE_RUMBLE;
	rumble.id = 0;
	rumble.intensity = state;
	msg.array.push_back(rumble);
	return msg;
}

// Receives a blank vector and fill it with the bullets data.
sensor_msgs::JoyFeedbackArray setLEDs(){
	sensor_msgs::JoyFeedbackArray msg;
	for(int ledid=0;ledid<MAXLED;ledid++){
		sensor_msgs::JoyFeedback led;
		led.type = sensor_msgs::JoyFeedback::TYPE_LED;
		led.id = ledid;
		led.intensity = bullets[ledid];
		msg.array.push_back(led);
	}
	return msg;
}

// A callback function . Executed each time a wiimote message arrives
void stateMessageReceived(const wiimote::State& msg){

	// SETTING GAME INIT
	if(!AtStart){
		if (isAllOn(msg.LEDs)){
			AtStart = true;
			numBullets = 4;
		}else{
			// Publish LED state (all turned on -- 4 initial bullets).
			LEDRumble_pub.publish(setLEDs());	// publish new LEDs state
		}
	}else{

		/* SHOTTING BUTTON */
		int newShootBtState = msg.buttons[5];								// Shoot button (back trigger button)
		if (prevShootBtState != newShootBtState){
			prevShootBtState = newShootBtState;

			// Create and fill in the message for arduino feedback (buzzer).
			robogame_wiimote_listener::State buttonmsg;						
			buttonmsg.shootButton = newShootBtState;
			buttonmsg.numBullets = numBullets;
			BUTTONpub.publish(buttonmsg);									// Publish the message.

			if ((newShootBtState == ON) && (numBullets > 0)){
				if(!((numBullets -1) < 0)) numBullets--;					// decrease bullets (a bullet used)
				updateBullets();											// update Bullets 
				LEDRumble_pub.publish(setLEDs());							// publish new wiimote LED state
			}else if ((newShootBtState == ON) && (numBullets == 0)){		// turns rumble on when there is no bullets left
				rumble = ON;
				LEDRumble_pub.publish(updateRumble(1));						// turn rumble on
			}else if ((newShootBtState == OFF) && (rumble == ON)){			// turns rumble off when the button is released
				rumble = OFF;
				LEDRumble_pub.publish(updateRumble(0));						// turn rumble off
			}
		}

		/* RECHARGE BUTTON */
		int newRecharBtState = msg.buttons[4];								// Recharge button (A button)
		if (prevRecharBtState != newRecharBtState){
			prevRecharBtState = newRecharBtState;
			if ((newRecharBtState == ON) && (numBullets < MAXLED)){
				numBullets++;												// increase #ofBullets
				updateBullets();											// update bullets vector
				LEDRumble_pub.publish(setLEDs());							// publish new LEDs state
			}else if ((newRecharBtState == ON) && (numBullets == MAXLED)){
				rumble = ON;
				LEDRumble_pub.publish(updateRumble(1));						// turn rumble on
			}else if ((newRecharBtState == OFF) && (rumble == ON)){
				rumble = OFF;
				LEDRumble_pub.publish(updateRumble(0));						// turn rumble off
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
	BUTTONpub = nh.advertise<robogame_wiimote_listener::State>("wiimote/fit_arduino_state",1000);		// advertise button state for arduino listener
	LEDRumble_pub = nh.advertise<sensor_msgs::JoyFeedbackArray>("/joy/set_feedback",1000);				// advertise LED and Rumble state

	ROS_INFO_STREAM("Listening to wiimote button event...");
	ROS_INFO_STREAM("This node controls LEDs and Rumble...");

	// Let ROS take over .
	ros::spin();
}
