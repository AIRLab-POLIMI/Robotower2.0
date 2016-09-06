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
 
#ifndef UTILS_H
#define UTILS_H

#include<wiimote/State.h>
#include "commons.h"

extern const int MAXLED;
extern const int ON;
extern const int OFF;
extern int bullets[4];

/* A callback function executed each time a wiimote message arrives in order
to get the target buttons from the controller.*/
void getTargetButtonState(const wiimote::State& msg, wiiButtons& buttons);

/* Checks if the wiimote has all LEDs on. */
bool isAllOn(const boost::array<unsigned char,4>  LEDsState);

/* Update LED state based on the number of bullets */
void updateBullets(int numBullets);

/* Update Rumble state */
sensor_msgs::JoyFeedbackArray updateRumble(float state);

/* Receives a blank vector and fill it with the bullets data. */
sensor_msgs::JoyFeedbackArray setLEDs();

#endif
