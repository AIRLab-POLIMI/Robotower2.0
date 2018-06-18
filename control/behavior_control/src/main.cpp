#include <ros/ros.h>
#include "behavior_control/planner.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "planner_node");
    
    Behavior::Planner b_manager;

    // Loop at 100Hz until the node is shutdown.
    ros::Rate rate(1);

    ROS_INFO("The Behavior Planner will wait 5secs before starting!");
    ros::Duration(5.0).sleep(); // sleep for 5 seconds before beginning.

    while(ros::ok()){
        b_manager.updateLoop();
        ros::spinOnce();
        // Wait until it's time for another iteration.
        rate.sleep() ;
    }
    return 0;
}