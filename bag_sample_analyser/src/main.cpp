#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <iostream>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH


int main(int argc, char *argv[]) {

	if (argc != 4 ){
		std::cout << "PARAMETER ERROR";
	}

	std::string input = argv[3];
	
	rosbag::Bag bag("/home/airlab/Scrivania/bags/clean/expa.bag");
	rosbag::View view(bag);
	std::vector<const rosbag::ConnectionInfo *> connection_infos = view.getConnections();
	std::set<std::string> topics;

	BOOST_FOREACH(const rosbag::ConnectionInfo *info, connection_infos) {
		//topics.insert(info->topic);
		std::cout << info->topic << " " << info->header << std::endl;
	}

	bag.close();
	return 0;
}

