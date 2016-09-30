// This program subscribes to wiimote and publishes the interested button state.
#include<ros/ros.h>
#include<wiimote/State.h>
#include<sensor_msgs/JoyFeedbackArray.h>
#include<sensor_msgs/JoyFeedback.h>
#include<robogame_game_manager/gameState.h>
#include <robogame_kinectfeatures_extractor/kinect_feat.h>
#include<time.h>
#include<cstdlib>

#include "commons.h"
#include "utils.h"

extern const int MAXLED = 4;									// Maximum LED number
extern const int ON = 1;
extern const int OFF = 0;
int prevShootBtState = OFF;
int prevRecharBtState = OFF;
int rumble = OFF;
int bullets[4] = {ON,ON,ON,ON};
int numBullets = 4;									            // four initial bullets available at game start
bool AtStart = false;
std::time_t previousTime = 0;
std::time_t currentTime;
double tooCloseInterval = 1;
float distanceThreshold = 1.10;
unsigned long previousMillis = 0;                               // will store last time LED was updated;

robogame_game_manager::gameState gameState;

// Create a publisher object for button state.
ros::Publisher gamepub;
// Create a LED publisher object.
ros::Publisher LEDRumble_pub;

/* MANAGES PLAYER POSITION BASED ON KINECT TOPICS*/
void handleKinFeatMessage(const robogame_kinectfeatures_extractor::kinect_feat& msg){
  std::time(&currentTime);
  if(msg.distance <= 0){
    gameState.isPlayerLost = true;
    gameState.blinkInterval = 250;
  }else if (!!gameState.isPlayerTooClose && (msg.distance < distanceThreshold) && (std::difftime(currentTime,previousTime) > tooCloseInterval)){
    gameState.isPlayerTooClose = true;
    gameState.blinkInterval = 125;
    gameState.isPlayerLost = false;
  }else if (msg.distance > distanceThreshold){
    std::time(&previousTime);
    gameState.isPlayerLost = false;
    gameState.isPlayerTooClose = false;
    gameState.blinkInterval = 250;
  }
}

/* MANAGES GAME STATE BASED ON THE WIIMOTE*/
void handleWiiMessage(const wiimote::State& msg){

	// SETTING GAME INIT
	if(!AtStart){
		if (isAllOn(msg.LEDs)){
			AtStart = true;
			numBullets = 4;                     // player has 4 bullets at the beggining.
		}else{
			// Publish LED state (all turned on -- 4 initial bullets).
			LEDRumble_pub.publish(setLEDs());	// publish new LEDs state
		}
	}else{

        /* GETTING WIIMOTE TARGET BUTTONS STATE*/
        wiiButtons buttons;
        getTargetButtonState(msg, buttons);
        
		/* MANAGING SHOTTING BUTTON */
		if (prevShootBtState != buttons.shoot){
			prevShootBtState = buttons.shoot;
			if ((buttons.shoot == ON) && (numBullets > 0) && ((msg.ir_tracking[0].x != -1) || ((msg.ir_tracking[0].y != -1)))){
				if(!((numBullets -1) < 0)){
				    numBullets--;					                        // decrease bullets (a bullet used)
				    gameState.beep = ON;                                    // make it beep to reflect bullet usage.
				}
				updateBullets(numBullets);									// update Bullets 
				LEDRumble_pub.publish(setLEDs());							// publish new wiimote LED state
			}else if ((buttons.shoot == ON) && (numBullets == 0)){		    // turns rumble on when there is no bullets left
				rumble = ON;
				LEDRumble_pub.publish(updateRumble(1));						// turn rumble on
			}else if ((buttons.shoot == OFF) && (rumble == ON)){			// turns rumble off when the button is released
				rumble = OFF;
				LEDRumble_pub.publish(updateRumble(0));						// turn rumble off
				gameState.beep = OFF;
			}else if (buttons.shoot == OFF){
			    gameState.beep = OFF;                                       // turning of beep;
			}
		}

		/* MANAGING RECHARGE BUTTON */
		if (prevRecharBtState != buttons.recharge){
			prevRecharBtState = buttons.recharge;
			if ((buttons.recharge == ON) && (numBullets < MAXLED)){
				numBullets++;												// increase #ofBullets
				updateBullets(numBullets);									// update bullets vector
				LEDRumble_pub.publish(setLEDs());							// publish new LEDs state
			}else if ((buttons.recharge == ON) && (numBullets == MAXLED)){
				rumble = ON;
				LEDRumble_pub.publish(updateRumble(1));						// turn rumble on
			}else if ((buttons.recharge == OFF) && (rumble == ON)){
				rumble = OFF;
				LEDRumble_pub.publish(updateRumble(0));						// turn rumble off
			}
		}
			
		/* PUBLISH GAME STATE*/ 
		gameState.numBullets = numBullets;

        gameState.isRecharger = false;
        gameState.startedAt   = 0;
        gameState.duration    = 0;

        gameState.shootButton = buttons.shoot;
        gameState.rechargeButton = buttons.recharge;

        gameState.LEDColorindex	= (((msg.ir_tracking[0].x != -1) || ((msg.ir_tracking[0].y != -1))) ? 0 : 1);   // LED set to red when the wiimote is pointed to the kinect
        gamepub.publish(gameState);
        /**/
	}
}

int main (int argc, char** argv){
	// Initialize the ROS system and become a node .
	ros::init(argc, argv, "game_manager");
	ros::NodeHandle nh;
	
	srand((int) time(0));

	// Create a subscriber object.
	ros::Subscriber kinsub = nh.subscribe("kinect_features",1000, &handleKinFeatMessage);
	ros::Subscriber wiisub = nh.subscribe("wiimote/state", 1000, &handleWiiMessage);
	gamepub = nh.advertise<robogame_game_manager::gameState>("robogame/gameState",1000);		        // advertise button state for arduino listener
	LEDRumble_pub = nh.advertise<sensor_msgs::JoyFeedbackArray>("/joy/set_feedback",1000);				// advertise LED and Rumble state
	
    //ROS_INFO_STREAM("Listening to wiimote button event...");
	//ROS_INFO_STREAM("This node controls LEDs and Rumble...");

	// Let ROS take over.
	ros::spin();
}
