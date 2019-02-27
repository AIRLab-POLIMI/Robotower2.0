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

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include "triskar/PixyPublisher.h"


class PixyTeleopJoy : public PixyPublisher
{
public:
	PixyTeleopJoy(ros::NodeHandle& nh) 
		: PixyPublisher(nh)
	{
		nh.param("reset_button", resetButton, -1);
		nh.param("axis_pan", panAxis, 1);
		nh.param("axis_tilt", tiltAxis, 2);
		nh.param("scale", scale, 1.0);

		joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 5, &PixyTeleopJoy::joyCallback, this);
	}
	
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
	{
		if(resetButton >= 0 && joy_msg->buttons[resetButton])
		{
			pan = (maxPan - minPan) / 2.0;
			tilt = (maxTilt - minTilt) / 2.0;
		}
		else
		{
			double panCommand = scale*joy_msg->axes[panAxis];
			double tiltCommand = scale*joy_msg->axes[tiltAxis];

			pan = bound(pan + panCommand, minPan, maxPan);
			tilt =  bound(tilt + tiltCommand, minTilt, maxTilt);
		}
	}

private:
	ros::Subscriber joy_sub;
	
	double scale;
	int resetButton;
	int panAxis, tiltAxis;
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "teleop_pixy_joy_node");

	ros::NodeHandle nh("~");
	PixyTeleopJoy joy_teleop(nh);

	while(ros::ok())
	{
		joy_teleop.spin();
	}
} 
