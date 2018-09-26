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
#include <geometry_msgs/Twist.h>
#include <triskar_msgs/Proximity.h>

class RandomExplorationNode
{
	enum State
	{
		Forward, Obstacle, Turn
	};

public:
	RandomExplorationNode(ros::NodeHandle& nh)
		: nh(nh), rate(20)
	{
		sonar_sub = nh.subscribe<triskar_msgs::Proximity>("/proximity", 5,
				&RandomExplorationNode::proximityCallback, this);

		cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);

		obstacleFront = true;
		obstacleLeft = true;
		obstacleRight = true;
		obstacleBack = true;

		state = Forward;
		time = ros::Time::now();
	}

	void proximityCallback(const triskar_msgs::Proximity::ConstPtr& prox_msg)
	{
		double threshold = 200;

		//look at the frontal sonars
		if(prox_msg->range[0] < threshold || prox_msg->range[4] < threshold)
			obstacleFront = true;
		else
			obstacleFront = false;

		//look at the left sonars
		if(prox_msg->range[1] < threshold || prox_msg->range[6] < threshold)
			obstacleLeft = true;
		else
			obstacleLeft = false;

		//look at the right sonars
		if(prox_msg->range[2] < threshold || prox_msg->range[5] < threshold)
			obstacleRight = true;
		else
			obstacleRight = false;

		//look at the right sonars
		if(prox_msg->range[3] < threshold || prox_msg->range[7] < threshold)
			obstacleBack = true;
		else
			obstacleBack = false;
	}

	void spin()
	{
		ros::spinOnce();
		stateTransition();

		geometry_msgs::Twist cmd_msg;

		switch (state)
		{
			case Forward:
				forwardState(cmd_msg);
				break;

			case Turn:
				turnState(cmd_msg);
				break;

			case Obstacle:
				obstacleState(cmd_msg);
				break;
		}


		cmd_pub.publish(cmd_msg);

		rate.sleep();
	}



	void forwardState(geometry_msgs::Twist& cmd_msg)
	{
		cmd_msg.linear.x = 0.5;
	}

	void obstacleState(geometry_msgs::Twist& cmd_msg)
	{
		cmd_msg.angular.z = 4.0;
	}

	void turnState(geometry_msgs::Twist& cmd_msg)
	{
		cmd_msg.linear.x = 0.5;

		if(!obstacleLeft)
			cmd_msg.angular.z = -4.0;
		else if(!obstacleRight)
			cmd_msg.angular.z = 4.0;
		else
			cmd_msg.angular.z = 0;
	}

	void stateTransition()
	{
		double durationForward = 5.0;
		double durationTurn = 1.0;
		double durationObstacle = 2.0;
		ros::Time stamp = ros::Time::now();
		ros::Duration delta = stamp - time;

		switch (state)
		{
			case Obstacle:
				if(!obstacleFront && (obstacleRight || obstacleLeft ||
						delta > ros::Duration(durationObstacle)))
				{
					state = Forward;
					time = stamp;
				}
				break;

			case Forward:
				if(obstacleFront)
				{
					state = Obstacle;
					time = stamp;
				}
				else if(delta > ros::Duration(durationForward))
				{
					state = Turn;
					time = stamp;
				}
				break;

			case Turn:
				if(obstacleFront)
				{
					state = Obstacle;
					time = stamp;
				}
				else if(stamp - time > ros::Duration(durationTurn))
				{
					state = Forward;
					time = stamp;
				}
				break;
		}
	}

private:
	ros::NodeHandle& nh;
	ros::Subscriber sonar_sub;
	ros::Publisher cmd_pub;
	ros::Rate rate;

	State state;
	ros::Time time;

	bool obstacleFront, obstacleLeft, obstacleRight, obstacleBack;

};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "random_exploration_node");

	ros::NodeHandle nh("~");
	RandomExplorationNode explorationNode(nh);

	while(ros::ok())
	{
		explorationNode.spin();
	}
}
