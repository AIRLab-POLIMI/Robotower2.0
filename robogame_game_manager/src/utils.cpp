/* commons.h -- This file is part of the robogame node created for
 * the purpose of extracting relevant motion features from images.
 *
 * Copyright (C) 2016 Ewerton Lopes
 *
 * LICENSE: This is a free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License (LGPL v3) as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version. This code is distributed in the hope
 * that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU Lesser General Public License (LGPL v3): http://www.gnu.org/licenses/.
 */
 
#include<sensor_msgs/JoyFeedbackArray.h>
#include<sensor_msgs/JoyFeedback.h>
#include<cstdlib>
#include "utils.h"
#include "commons.h"
#include<time.h>

bool isRechargePause = false;
float rechargeDuration = 0;
float rechargePauseDuration = 0;


/* A callback function executed each time a wiimote message arrives in order
to get the target buttons from the controller.*/
void getTargetButtonState(const wiimote::State& msg, wiiButtons& buttons){
        /*   msg.button[5] - Shoot button (back trigger button)
		     msg.button[4] - Recharge button (A button) */
		buttons.shoot = msg.buttons[5];
		buttons.recharge = msg.buttons[4];
}

/* Checks if the wiimote has all LEDs on. */
bool isAllOn(const boost::array<unsigned char,4>  LEDsState){
	int count=0;
	for(int i=0;i<MAXLED;i++){
		count += LEDsState[i];
	}
	return (count == MAXLED);
}

/* Update LED state based on the number of bullets */
void updateBullets(int numBullets){
	for(int i=0;i<MAXLED;i++){
		if(i < numBullets){
			bullets[i] = ON;
		}else{
			bullets[i] = OFF;
		}
	}
}

/* Update Rumble state */
sensor_msgs::JoyFeedbackArray updateRumble(float state){
	sensor_msgs::JoyFeedbackArray msg;
	sensor_msgs::JoyFeedback rumble;
	rumble.type = sensor_msgs::JoyFeedback::TYPE_RUMBLE;
	rumble.id = 0;
	rumble.intensity = state;
	msg.array.push_back(rumble);
	return msg;
}

/* Receives a blank vector and fill it with the bullets data. */
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


/*
std::time_t pauseInit;
std::time_t currentTime;

enum eventType {RECHARGEON, RECHARGEOFF};
int eventchoice[2] = {RECHARGEON, RECHARGEOFF};*/

/* A function to generate a recharging period for bullets*/
float generateRechargeTime(float probability){
    /*/* This function is supposed to say when the recharge time is on.
     IDEA:
            - decise if you are going to enter the recharge time
            - choose a duration.
            - choose a pause time.
            - repeat.
   
    
    if (!isRechargePause){
        int divisor = 10000;
        float duration = (std::rand() % divisor);
        float prob = ((double) std::rand() / RAND_MAX) +1;
        if (prob < probability){
            return duration;
        }
    }else if (rechargePauseDuration){
        std::time(&currentTime);
        
    }
    
    bool hasEvent = false;
    
    
        
    // choose pause duration
    if (!hasEvent){
        int eventChoosen = eventchoice[((double) std::rand() / RAND_MAX) +1];
        hasEvent = !hasEvent;
        if (eventChoosen == ev
        float pauseDuration = (std::rand() % 60000);
        std::time(&pauseInit);
        isRechargeInPause = true;
    }else if (isRechargeInPause){
        std::time(&currentTime);
        if (std::difftime(currentTime,pauseInit) > pauseDuration){
            isRechargeInPause = false;
        }
    }else if (isRechargeTime){
        
    } */
    
}
