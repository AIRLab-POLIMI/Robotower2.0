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

#include <triskar_msgs/Pixy.h>

#include "triskar/PixyPublisher.h"

class PixyTracker : public PixyPublisher
{

	struct controllerState
	{
		int Kp;
		int Kd;
		int error;
		int max;
		int min;
	};

public:
	PixyTracker(ros::NodeHandle& nh)
		: PixyPublisher(nh)
	{
		panController.error = 0;
		tiltController.error = 0;

		nh.param("panKp", panController.Kp, 400);
		nh.param("panKd", panController.Kd, 300);
		nh.param("tiltKp", tiltController.Kp, 500);
		nh.param("tiltKd", tiltController.Kd, 400);

		nh.param("signatureId", signatureId, 1);

		centerX = (maxX - minX)/2;
		centerY = (maxY - minY)/2;

		panController.max = maxPan;
		panController.min = minPan;
		tiltController.max = maxTilt;
		tiltController.min = minTilt;

		pixy_sub = nh.subscribe<triskar_msgs::Pixy>("/pixy", 5, &PixyTracker::pixyCallback, this);
	}

	void pixyCallback(const triskar_msgs::Pixy::ConstPtr& pixy_msg)
	{
		if(pixy_msg->signature == signatureId)
		{
			update(centerX - pixy_msg->x, panController, pan);
			update(pixy_msg->y - centerY, tiltController, tilt);
		}
	}

	void update(int error, controllerState& state, double& axis)
	{
		int error_delta = state.error - error;

		int velocity = static_cast<int>(state.Kp*error + state.Kd*error_delta) >> 10;

	    int position = axis + velocity;

		axis = bound(position, state.min, state.max);

		state.error = error;
	}


private:
	ros::Subscriber pixy_sub;

	controllerState panController;
	controllerState tiltController;

	int centerX, centerY;

	int signatureId;
};


int main(int argc, char *  argv[])
{
	ros::init(argc, argv, "teleop_tracker_node");

	ros::NodeHandle nh("~");
	PixyTracker tracker(nh);

	while(ros::ok())
	{
		tracker.spin();
	}

    return 0;
}



