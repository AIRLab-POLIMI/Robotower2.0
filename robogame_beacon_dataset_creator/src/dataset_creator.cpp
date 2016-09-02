// This program subscribes to wiimote and publishes the interested button state.
#include<ros/ros.h>
#include<wiimote/State.h>
#include<wiimote/IrSourceInfo.h>
#include<robogame_rosarduino/State.h>
#include <iostream>
#include <fstream>
#include <ctime>
#include <vector>

#include <string>       // std::string
#include <sstream>      // std::stringstream


// The wiimote limits to 4 the number of ir LEDs it can see.
const int MAX_IR_LED = 4;
//output file
std::ofstream output;
std::string dependentVariable;

// A callback function . Executed each time a wiimote message arrives
void stateMessageReceived(const wiimote::State& msg){
	std::vector<wiimote::IrSourceInfo> ir_tracked = msg.ir_tracking;
	for (int i=0; i<MAX_IR_LED; i++){
		wiimote::IrSourceInfo ir = ir_tracked[i];
		output << ir.x << ',';
		output << ir.y << ',';
	}
	output << dependentVariable << std::endl;
}

int main (int argc, char** argv){
	// Initialize the ROS system and become a node .
	ros::init(argc, argv, "beacon_dataset_creator");
	ros::NodeHandle nh;
	
	if (argc < 2){
		std::cout << "USAGE: argument error!" << std::endl;
		exit(-1);	
	}
	dependentVariable = argv[1];

	std::stringstream filename;
	filename << "/home/airlab/catkin_ws/src/phd_robogame/robogame_beacon_dataset_creator/dataset/";
	std::time_t t = std::time(0);   // get time now
    struct std::tm * now = std::localtime( & t );
    filename << (now->tm_year + 1900) << '-' 
         << (now->tm_mon + 1) << '-'
         << now->tm_mday << '_'
         << now->tm_hour << '-'
		 << now->tm_min << '-'
		 << now->tm_sec << ".txt";
	
	output.open(filename.str().c_str());
 
	// Create a subscriber object.
	ros::Subscriber sub = nh.subscribe("wiimote/state", 1000, &stateMessageReceived);
	
	ROS_INFO_STREAM("Saving data set...");

	// Let ROS take over .
	ros::spin();
}
