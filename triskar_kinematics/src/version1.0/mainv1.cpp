#include<ros/ros.h>
#include<ros/console.h>
#include "TriskarForward.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "TriskarForward_kinematics");
	TriskarForward forward_kinematics;

	ros::spin();
	return 0;
}
