/*
 * triskar,
 *
 *
 * Copyright (C) 2015 Davide Tateo & Matteo Pirotta
 * Versione 1.0
 *
 * This file is part of triskar.
 *
 * triskar is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * triskar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with triskar.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "triskar/PixyPublisher.h"
#include <triskar_msgs/PixyServo.h>
#include <cmath>

PixyPublisher::PixyPublisher(ros::NodeHandle& nh)
	: nh(nh), rate(0)
{
	double freq;
	nh.param("rate", freq, 50.0);
	rate = ros::Rate(freq);

	nh.param("minPan", minPan, 0);
	nh.param("minTilt", minTilt, 0);
	nh.param("maxPan", maxPan, 1000);
	nh.param("maxTilt", maxTilt, 1000);

	nh.param("minX", minX, 0);
	nh.param("minY", minY, 0);
	nh.param("maxX", maxX, 319);
	nh.param("maxY", maxY, 199);

	pan = (maxPan - minPan) / 2.0;
	tilt = (maxTilt - minTilt) / 2.0;

	cmd_pub = nh.advertise<triskar_msgs::PixyServo>("/pixy_servo", 5);

}

double PixyPublisher::bound(double value, int min, int max)
{
	if(value < min)
		return min;
	else if(value > max)
		return max;
	else
		return value;
}

void PixyPublisher::spin()
{
	triskar_msgs::PixyServo cmd_msg;
	cmd_msg.pan = std::ceil(pan);
	cmd_msg.tilt = std::ceil(tilt);
	cmd_pub.publish(cmd_msg);

	ros::spinOnce();
	rate.sleep();
}


