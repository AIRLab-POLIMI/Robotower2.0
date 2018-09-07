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
    // if(desired_vel.x == 0 && desired_vel.y == 0 && desired_vel.z == 0){
    //     return output;
    // }
    output = VectorUtility::vector_difference(current_vel, desired_vel);
    return output;
}

bool SteeringBehavior::Flee::evaluateSafety(geometry_msgs::Point32 current_pos){
    float distance_from_target = VectorUtility::distance(current_pos, target_);
    // ROS_WARN("We're %f meters from target", distance_from_target);
    if(distance_from_target > safety_distance_){
        return true;
    }
    return false;
}


