/*
 * PixyPublisher.h
 *
 *  Created on: 22 nov 2016
 *      Author: dave
 */

#ifndef INCLUDE_TRISKAR_PIXYPUBLISHER_H_
#define INCLUDE_TRISKAR_PIXYPUBLISHER_H_

#include <ros/ros.h>

class PixyPublisher
{
public:
	PixyPublisher(ros::NodeHandle& nh);
	void spin();

protected:
	static double bound(double value, int min, int max);

private:
	ros::NodeHandle& nh;
	ros::Publisher cmd_pub;
	ros::Rate rate;

protected:
	double pan, tilt;
	int minPan, maxPan,	minTilt, maxTilt;
	int minX, minY, maxX, maxY;
};



#endif /* INCLUDE_TRISKAR_PIXYPUBLISHER_H_ */
