#include "planning2/behaviors/seek.h"


geometry_msgs::Vector3 LocomotionPlanning::Seek::calculate_desired_velocity(geometry_msgs::Point32 current_pos){
    geometry_msgs::Vector3 output;
    output = VectorUtility::vector_difference(current_pos, target_);
    output = VectorUtility::normalize(output);
    output = VectorUtility::scalar_multiply(output, max_speed_);
    return output;
}

geometry_msgs::Vector3 LocomotionPlanning::Seek::calculate_steering_force(geometry_msgs::Vector3 current_vel, geometry_msgs::Vector3 desired_vel){
    geometry_msgs::Vector3 output;
    
    output = VectorUtility::vector_difference(current_vel, desired_vel);
    return output;
}

bool LocomotionPlanning::Seek::targetReached(geometry_msgs::Point32 current_pos){
    // If we're close (given a threshold) to the target return true
    return false;
}
    		
void LocomotionPlanning::Seek::updateTargetPos(geometry_msgs::Point32 newTargetPos){
    // Updates the position of the target
}

float LocomotionPlanning::Seek::evaluate(double player_model){
    return 2;
}
