#include <ros/ros.h>
#include "basic_navigation/navigation.h"

int main(int argc, char** argv){

    ros::init(argc, argv, "basic_navigation_node");
    ros::NodeHandle nh;
    ros::Rate rate(20);

    Navigation::BasicNavigation navigation;

    while(ros::ok()){
        // planner.updateLoop();
        navigation.run();
        ros::spinOnce();
        rate.sleep();
    }
}
