#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <nav_msgs/Odometry.h>
#include "behavior_manager/behavior_manager.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "behavior_manager");
    
    BehaviorManager b_manager;

    // Loop at 100Hz until the node is shutdown.
    ros::Rate rate(1);

    ros::Duration(5.0).sleep(); // sleep for 5 seconds before beginning.

    while(ros::ok()){
        b_manager.update_loop();
        ros::spinOnce();
        // Wait until it's time for another iteration.
        rate.sleep() ;
    }
    return 0;
}