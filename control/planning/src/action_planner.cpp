#include <ros/ros.h>
#include <cmath>
#include "planning/action_planning.h"
#include "planning/ActionEncoded.h"
#include "planning/SafetyMsg.h"

#define CAPTURE_TOWER 1
#define ESCAPE 2

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


    // scan_sub_ = nh_.subscribe("/scan", 1, &ActionPlanning::ActionPlanner::laserCallback, this);
    // player_angle_sub_ = nh_.subscribe(player_angle_topic_, 1, &ActionPlanning::ActionPlanner::playerCallback, this);
    // vel_sub_ = nh_.subscribe(vel_topic_, 1, &ActionPlanning::ActionPlanner::velCallback, this);

    
    action_pub_ = nh_.advertise<planning::ActionEncoded>(action_topic_, 1);
    safety_sub_ = nh_.subscribe(safety_topic_, 1, &ActionPlanning::ActionPlanner::safetyCallback, this);

    safety_msg_.safety = true;

    silly_counter_ = 0;
}

void ActionPlanning::ActionPlanner::laserCallback(const sensor_msgs::LaserScan& scan){
    // Segment surrounding of robot into sections
    // Useful for safety calculations
    int number_of_sections = 8;
    int section_lenght = 1000 / number_of_sections;
    current_scan_segmented_.resize(number_of_sections);
    int i;
    int j;
    int k;
    for(i=0; i<number_of_sections; i++){
        j = i * section_lenght;
        current_scan_segmented_[i].resize(section_lenght);
        for(k=0; k < section_lenght; k++){
            current_scan_segmented_[i].push_back(scan.ranges[j+k]);
        }
    }
    current_scan_ = scan;
}

void ActionPlanning::ActionPlanner::velCallback(const geometry_msgs::Twist& vel){
    current_angle_motion_ = atan2(vel.linear.y, vel.linear.x);
    ROS_ERROR("Current motion angle: %.2f", current_angle_motion_ * (180/3.14));
}

bool ActionPlanning::ActionPlanner::evaluateSafety(){
    // Function to evaluate wheter the navigation is safe or not

    // Check wheter the player is in the motion direction
    // bool is_colliding_direction = isCollidingDirection();
    // if(is_colliding_direction){
    //     // Identify the one we're moving towards
    //     // Check if there's something close to it
    //     // If so, we're NOT safe
    //     // Else we're safe

    // }
    // else{
    //     // Player is not in the direction of movement
    //     return false;
    // }
    return true;
    if(silly_counter_ == 3){
        return true;
    }
    else if(silly_counter_ == 8){
        return false;
    }
}

void ActionPlanning::ActionPlanner::safetyCallback(const planning::SafetyMsg& msg){
    safety_msg_ = msg;
    // ROS_INFO("Safety: %", msg.data);
}

void ActionPlanning::ActionPlanner::updateLoop(){
    planning::ActionEncoded action_msg;

    bool is_safe = safety_msg_.safety;
    if(is_safe){

        action_msg.priority = silly_priority_;
        action_msg.action_name = "capture_tower";
        action_msg.action_code = CAPTURE_TOWER;
    
        action_pub_.publish(action_msg);
    }
    else{
        ROS_WARN("ESCAPING");
        action_msg.priority = silly_priority_;
        action_msg.action_name = "escape";
        action_msg.action_code = ESCAPE;
        action_msg.danger_sector = safety_msg_.section_index;
        action_msg.danger_index = safety_msg_.danger_index;

        action_pub_.publish(action_msg);
        silly_counter_ = 0;
    }

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