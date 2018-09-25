#include <ros/ros.h>
#include <string.h>
#include <std_msgs/Bool.h>

#include "planning/ActionEncoded.h"

#define CAPTURE_TOWER 1
#define ESCAPE 2
#define DECEIVE 3

int main(int argc, char** argv){
    ros::init(argc, argv, "planning_testing_node");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    std::string action_topic_;
    if (!nh.getParam("/planning/action_topic", action_topic_)){
        ROS_ERROR("ACTION PLANNER: could not read 'action_topic' from rosparam!");
        exit(-1);
    }

    // ros::Publisher action_pub_;
    // action_pub_ = nh.advertise<planning::ActionEncoded>(action_topic_, 1);

    ros::Publisher deception_pub_;
    deception_pub_ = nh.advertise<std_msgs::Bool>("/deception", 1);
    // planning::ActionEncoded action_msg;
    std_msgs::Bool deception;
    deception.data = true;

    // action_msg.priority = 1.0;
    // action_msg.action_name = "deceive";
    // action_msg.action_code = DECEIVE;
    int counter = 0;
    
    while(ros::ok()){
        // planner.updateLoop();
        // action_pub_.publish(action_msg);
        // ROS_INFO("Testing deception...");
        // if(counter > 30){
        //     deception.data = false;
        // }
        deception_pub_.publish(deception);

        counter++;
        // ROS_ERROR("COUNT: %d", counter);
        ros::spinOnce();
        rate.sleep();
    }
}