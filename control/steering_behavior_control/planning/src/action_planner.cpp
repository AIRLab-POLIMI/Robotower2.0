#include <ros/ros.h>
#include <cmath>
#include "planning/action_planning.h"
#include "planning/ActionEncoded.h"
#include "planning/SafetyMsg.h"

#define CAPTURE_TOWER 1
#define ESCAPE 2
#define DECEIVE 3

ActionPlanning::ActionPlanner::ActionPlanner(){
    ROS_INFO("Creating action planner...");

    if (!nh_.getParam("/planning/action_topic", action_topic_)){
            ROS_ERROR("ACTION PLANNER: could not read 'action_topic' from rosparam!");
            exit(-1);
    }

    if (!nh_.getParam("/planning/vel_topic_sub", vel_topic_)){
            ROS_ERROR("ACTION PLANNER: could not read 'vel_topic _sub' from rosparam!");
            exit(-1);
    }

    if (!nh_.getParam("/planning/player_angle_topic", player_angle_topic_)){
        ROS_ERROR("ACTION PLANNER: could not read 'player_angle_topic' from rosparam!");
        exit(-1);
    }

    if (!nh_.getParam("/planning/safety_topic", safety_topic_)){
        ROS_ERROR("SAFETY MANAGER: could not read 'player_angle_topic' from rosparam!");
        exit(-1);
    }
    if (!nh_.getParam("/planning/deception_topic", deception_topic_)){
        ROS_ERROR("SAFETY MANAGER: could not read 'player_angle_topic' from rosparam!");
        exit(-1);
    }
    
    action_pub_ = nh_.advertise<planning::ActionEncoded>(action_topic_, 1);
    safety_sub_ = nh_.subscribe(safety_topic_, 1, &ActionPlanning::ActionPlanner::safetyCallback, this);
    deception_sub_ = nh_.subscribe(deception_topic_, 1, &ActionPlanning::ActionPlanner::deceptionCallback, this);

    safety_msg_.safety = true;
    // deceiving_msg_.being_deceptive = false;
    deceiving_msg_.data = false;

    silly_counter_ = 0;
}

void ActionPlanning::ActionPlanner::safetyCallback(const planning::SafetyMsg& msg){
    safety_msg_ = msg;
    // ROS_INFO("Safety: %", msg.data);
}

void ActionPlanning::ActionPlanner::deceptionCallback(const std_msgs::Bool& msg){
    ROS_INFO("DECEIVING");
    deceiving_msg_ = msg;
    // ROS_INFO("Safety: %", msg.data);
}

void ActionPlanning::ActionPlanner::updateLoop(){
    planning::ActionEncoded action_msg;

    bool is_safe = safety_msg_.safety;
    // bool is_deceiving = deceiving_msg_.being_deceptive;
    bool is_deceiving = deceiving_msg_.data;
    if(!is_safe){
        ROS_WARN("ESCAPING");
        action_msg.priority = silly_priority_;
        action_msg.action_name = "escape";
        action_msg.action_code = ESCAPE;
        action_msg.danger_sector = safety_msg_.section_index;
        action_msg.danger_index = safety_msg_.danger_index;

        
        silly_counter_ = 0;
        
    }
    else if(is_deceiving){
        action_msg.priority = silly_priority_;
        action_msg.action_name = "deceive";
        action_msg.action_code = DECEIVE;
    }
    else{
        action_msg.priority = silly_priority_;
        action_msg.action_name = "capture_tower";
        action_msg.action_code = CAPTURE_TOWER;
    
        // action_pub_.publish(action_msg);
    }
    action_pub_.publish(action_msg);

    silly_counter_++;
    silly_priority_++;

}

int main(int argc, char** argv){
    ros::init(argc, argv, "planning_action_node");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    ActionPlanning::ActionPlanner planner;

    while(ros::ok()){
        planner.updateLoop();
        // ROS_INFO("Planning actions...");
        ros::spinOnce();
        rate.sleep();
    }
}
