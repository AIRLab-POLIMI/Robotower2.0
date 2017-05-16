#include <sstream>
#include <vector>

#include "ros/ros.h"
#include "heartbeat/HeartbeatServer.h"
#include "heartbeat/SetState.h"
#include "heartbeat/RegisterNode.h"
#include "heartbeat/UnregisterNode.h"


int main(int argc, char **argv) {

	ros::init(argc, argv, "heartbeat_server");

	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	HeartbeatServer hb(n);
	hb.start();

	while (ros::ok()) {
		hb.spin();

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}
