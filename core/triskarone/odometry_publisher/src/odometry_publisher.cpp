/*********************************************************************
*
* Software License Agreement (BSD License)
*
*	Copyright (c) 2009, Willow Garage, Inc.
*	All rights reserved.
*
*	Redistribution and use in source and binary forms, with or without
*	modification, are permitted provided that the following conditions
*	are met:
*
*	 * Redistributions of source code must retain the above copyright
*		 notice, this list of conditions and the following disclaimer.
*	 * Redistributions in binary form must reproduce the above
*		 copyright notice, this list of conditions and the following
*		 disclaimer in the documentation and/or other materials provided
*		 with the distribution.
*	 * Neither the name of Willow Garage, Inc. nor the names of its
*		 contributors may be used to endorse or promote products derived
*		 from this software without specific prior written permission.
*
*	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*	POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

//#define ADJUSTING_FACTOR 0.934579439


ros::Subscriber velSub;
ros::Publisher odomPub;
tf::TransformBroadcaster* odomBroadcaster;

ros::Time currentTime, lastTime;
double x = 0.0;
double y = 0.0;
double th = 0.0;

void velCallback(const geometry_msgs::Twist::ConstPtr& msg){
	
	double vx = msg->linear.x;
	double vy = msg->linear.y;
	double vth = msg->angular.z;
	
	currentTime = ros::Time::now();
	
	// compute odometry in a typical way given the velocities of the robot
	double dt = (currentTime - lastTime).toSec();
	double delta_th = vth * dt;
	th += delta_th;
	
	double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
	double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
	
	
	x += delta_x;
	y += delta_y;
	
	
	// since all odometry is 6DOF we'll need a quaternion created from yaw
	geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(th);
	
	// first, we'll publish the transform over tf
	geometry_msgs::TransformStamped odomTrans;
	odomTrans.header.stamp = currentTime;
	odomTrans.header.frame_id = "odom";
	odomTrans.child_frame_id = "base_link";
	
	odomTrans.transform.translation.x = x;
	odomTrans.transform.translation.y = y;
	odomTrans.transform.translation.z = 0.0;
	odomTrans.transform.rotation = odomQuat;
	
	// send the transform
	odomBroadcaster->sendTransform(odomTrans);
	
	// next, we'll publish the odometry message over ROS
	nav_msgs::Odometry odom;
	odom.header.stamp = currentTime;
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_link";
	
	//set the position
	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odomQuat;
	
	//set the velocity
	odom.twist.twist.linear.x = vx;
	odom.twist.twist.linear.y = vy;
	odom.twist.twist.angular.z = vth;
	
	//publish the message
	odomPub.publish(odom);
	
	lastTime = currentTime;
}


int main(int argc, char** argv){
	ros::init(argc, argv, "odometry_publisher");
	
	ros::NodeHandle n;
	
	velSub = n.subscribe("vel", 1000, velCallback);
	odomPub = n.advertise<nav_msgs::Odometry>("odom", 100);
	odomBroadcaster = new tf::TransformBroadcaster();
	
	currentTime = ros::Time::now();
	lastTime = ros::Time::now();
	
	ros::spin();
	
}
