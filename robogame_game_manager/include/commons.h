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

#ifndef COMMON_H
#define COMMON_H

extern const int MAXLED;        // maximum number of LEDs on wiimote.
extern int bullets[4];          // keeps track of the bullets status (Corresponds to the LEDs on the wiimote)
extern const int ON;
extern const int OFF;

typedef struct {
    bool shoot = 0;
    bool recharge = 0;   
}wiiButtons;

#endif
