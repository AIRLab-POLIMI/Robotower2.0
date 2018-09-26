#include "steering_behavior/flee.h"
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>

geometry_msgs::Vector3 SteeringBehavior::Flee::calculate_desired_velocity(geometry_msgs::Point32 current_pos){
    geometry_msgs::Vector3 output;
    output = VectorUtility::vector_difference(target_, current_pos);
    return output;
}

geometry_msgs::Vector3 SteeringBehavior::Flee::calculate_steering_force(geometry_msgs::Vector3 current_vel, geometry_msgs::Vector3 desired_vel){
    geometry_msgs::Vector3 output;
    output = VectorUtility::vector_difference(current_vel, desired_vel);
    return output;
}

bool SteeringBehavior::Flee::evaluateSafety(geometry_msgs::Point32 current_pos){
    float distance_from_target = VectorUtility::distance(current_pos, target_);
    if(distance_from_target > safety_distance_){
        return true;
    }
    return false;
}

float SteeringBehavior::Flee::evaluate(){
    return 0.0;
}

bool SteeringBehavior::Flee::updateTarget(sensor_msgs::LaserScan scan, geometry_msgs::Point32 current_pos, float current_rotation_wrt_map){
    float min_dist = 6.0; // minimum distance from sensor is 5.6 meters
    int min_index;

    for(int i=0; i<1000; i++){
        if(scan.ranges[i] < min_dist){
            min_dist = scan.ranges[i];
            min_index = i;
        }
    }

    ROS_ERROR("Escaping from %d:", min_index);

    float obstacle_angle = min_index * (2*M_PI/1000);

    geometry_msgs::Point32 target;
    target.x = current_pos.x + cos(obstacle_angle + current_rotation_wrt_map + M_PI)*min_dist;
    target.y = current_pos.y + sin(obstacle_angle + current_rotation_wrt_map + M_PI)*min_dist;

    setTarget(target);

    if(to_update_){
        to_update_ = false;
        return true;
    }

    return false;
}


