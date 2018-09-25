#include <ros/ros.h>
#include "planning/safety_evaluator.h"
#include "planning/SafetyMsg.h"
#include <std_msgs/Bool.h>

Safety::SafetyEvaluator::SafetyEvaluator(){
    if (!nh_.getParam("/planning/vel_topic_sub", vel_topic_)){
            ROS_ERROR("SAFETY PLANNER: could not read 'vel_topic_sub' from rosparam!");
            exit(-1);
    }

    if (!nh_.getParam("/planning/player_angle_topic", player_angle_topic_)){
        ROS_ERROR("SAFETY PLANNER: could not read 'player_angle_topic' from rosparam!");
        exit(-1);
    }

    if (!nh_.getParam("/planning/safety_treshold", safety_treshold_)){
        ROS_ERROR("SAFETY PLANNER: could not read 'safety_treshold' from rosparam!");
        exit(-1);
    }

    if (!nh_.getParam("/planning/number_sections_safety", number_of_sections_)){
        ROS_ERROR("SAFETY PLANNER: could not read 'number_sections_safety' from rosparam!");
        exit(-1);
    }

    if (!nh_.getParam("/planning/safety_topic", safety_topic_)){
        ROS_ERROR("SAFETY MANAGER: could not read 'safety_topic' from rosparam!");
        exit(-1);
    }

    laser_sub_ = nh_.subscribe("/scan", 1, &Safety::SafetyEvaluator::laserCallback, this);
    player_angle_sub_ = nh_.subscribe(player_angle_topic_, 1, &Safety::SafetyEvaluator::playerCallback, this);
    vel_sub_ = nh_.subscribe(vel_topic_, 1, &Safety::SafetyEvaluator::velCallback, this);
    safety_pub_ = nh_.advertise<planning::SafetyMsg>(safety_topic_, 1);

    section_angular_width_ = 360 / number_of_sections_;
    currently_safe_ = true;

    count_ = 0;
    other_count_ = 0;

    current_heading_sector_ = 0;
    current_player_sector_ = 1;
}

bool Safety::SafetyEvaluator::isCollidingDirection(){
    if(current_heading_sector_ == current_player_sector_){
        return true;
    }
    
    return false;
}

bool Safety::SafetyEvaluator::checkAllScan(){
    // Check the whole scan if there's something below treshold
    float min_dist = 6.0;
    for(int i=0; i<1000; i++){
        if(current_scan_.ranges[i] < min_dist){
            min_dist = current_scan_.ranges[i];
        }
    }

    if(min_dist <= safety_treshold_){
        // We are in a non safe condition
        ROS_WARN("UNSAFE");
        return false;
    }
    else{
        return true;
    }
}

bool Safety::SafetyEvaluator::checkHeadingSector(){
    // Checks if there's something too close in the direction of movement
    std::vector<float> sector_ranges;
    sector_ranges = current_scan_segmented_[current_heading_sector_];

    float min_dist = 6.0;
    int min_index = 0;
    int section_lenght = 1000 / number_of_sections_;

    for(int i=0; i<section_lenght; i++){
        if(sector_ranges[i] < min_dist){
            min_dist = sector_ranges[i];
            min_index = i;
        }
    }
    ROS_INFO("Min dist: %.2f", min_dist);
    if(min_dist <= safety_treshold_){
        // We are in a non safe condition
        ROS_WARN("UNSAFE");
        return false;
    }
    else{
        return true;
    }
}

bool Safety::SafetyEvaluator::evaluateSafety(){
    // Function to evaluate wheter the navigation is safe or not

    // Check wheter the player is in the motion direction
    if(!currently_safe_){
        currently_safe_ = checkAllScan();
    }
    else{
        ROS_WARN("Checking movement direction");
        bool is_colliding_direction = isCollidingDirection();
        if(is_colliding_direction){
            ROS_WARN("WE'RE MOVING TOWARDS PLAYER");
            // Identify the one we're moving towards
            // Check if there's something close to it
            // If so, we're NOT safe
            // Else we're safe
            currently_safe_ = checkHeadingSector();
        }
    }
    

    safety_msg_.safety = currently_safe_;
    safety_msg_.section_index = current_heading_sector_;

    safety_pub_.publish(safety_msg_);
    return currently_safe_;
}

void Safety::SafetyEvaluator::laserCallback(const sensor_msgs::LaserScan& scan){
    // Segment surrounding of robot into sections
    // Useful for safety calculations

    int section_lenght = scan.ranges.size() / number_of_sections_;
    current_scan_segmented_.resize(number_of_sections_);
    int i;
    int j;
    int k;
    for(i=0; i<number_of_sections_; i++){
        j = i * section_lenght;
        current_scan_segmented_[i].resize(section_lenght);
        for(k=0; k < section_lenght; k++){
            current_scan_segmented_[i][k] = scan.ranges[j+k];
        }
    }
    current_scan_ = scan;
}

void Safety::SafetyEvaluator::velCallback(const geometry_msgs::Twist& vel){
    current_angle_motion_ = atan2(vel.linear.y, vel.linear.x);
    int current_angle_deg = current_angle_motion_ * (180 / M_PI); // Converting to degrees
    current_heading_sector_ = (current_angle_deg + 180) / section_angular_width_;
    ROS_ERROR("Current motion angle: %.2f", current_angle_motion_ * (180/M_PI));
    ROS_ERROR("Current heading sector: %d", current_heading_sector_);
}

void Safety::SafetyEvaluator::playerCallback(const geometry_msgs::PointStamped& position){
    current_angle_player_ = atan2(position.point.y, position.point.x);
    int current_angle_deg = current_angle_player_ * (180 / M_PI);  // Converting to degrees
    current_player_sector_ = (current_angle_deg + 180) / section_angular_width_;
    ROS_ERROR("Current player angle: %.2f", current_angle_player_ * (180/M_PI));
    ROS_ERROR("Current player sector: %d", current_player_sector_);
}


int main(int argc, char** argv){
    ros::init(argc, argv, "planning_safety_node");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    Safety::SafetyEvaluator safety_evaluator;

    while(ros::ok()){
        safety_evaluator.evaluateSafety();
        ros::spinOnce();
        rate.sleep();
    }
}