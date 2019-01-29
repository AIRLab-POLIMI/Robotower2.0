#include "planning2/behaviors/arrival.h"


geometry_msgs::Vector3 LocomotionPlanning::Arrival::calculate_desired_velocity(geometry_msgs::Point32 current_pos){
    geometry_msgs::Vector3 output;
    float distance_from_target;
    float ramped_speed;
    float clipped_speed;
    distance_from_target = VectorUtility::distance(target_, current_pos);

    ramped_speed = max_speed_ * (distance_from_target / slowing_radius_);
    clipped_speed = std::min(ramped_speed, max_speed_);
    
    output = VectorUtility::vector_difference(current_pos, target_);
    output = VectorUtility::scalar_multiply(output, (clipped_speed / distance_from_target));

    return output;
}

geometry_msgs::Vector3 LocomotionPlanning::Arrival::calculate_steering_force(geometry_msgs::Vector3 current_vel, geometry_msgs::Vector3 desired_vel){
    geometry_msgs::Vector3 output;
    
    output = VectorUtility::vector_difference(current_vel, desired_vel);
    return output;
}

bool LocomotionPlanning::Arrival::targetReached(geometry_msgs::Point32 current_pos){
    // If we're close (given a threshold) to the target return true
    return false;
}
    		
void LocomotionPlanning::Arrival::updateTargetPos(geometry_msgs::Point32 newTargetPos){
    // Updates the position of the target
}

float LocomotionPlanning::Arrival::evaluate(double player_model){
    return 4;
}
