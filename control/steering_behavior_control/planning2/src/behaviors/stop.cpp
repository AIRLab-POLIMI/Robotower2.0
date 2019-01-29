#include "planning2/behaviors/stop.h"

geometry_msgs::Vector3 LocomotionPlanning::Stop::calculate_desired_velocity(geometry_msgs::Point32 current_pos){
    geometry_msgs::Vector3 output;
    // output = VectorUtility::revert(current_vel);
    return output;
}

geometry_msgs::Vector3 LocomotionPlanning::Stop::calculate_steering_force(geometry_msgs::Vector3 current_vel, geometry_msgs::Vector3 desired_vel){
    geometry_msgs::Vector3 output;
    
    output = VectorUtility::vector_difference(current_vel, VectorUtility::revert(current_vel));
    return output;
}

bool LocomotionPlanning::Stop::targetReached(geometry_msgs::Point32 current_pos){
    // If we're close (given a threshold) to the target return true
    return false;
}
    		
void LocomotionPlanning::Stop::updateTargetPos(geometry_msgs::Point32 newTargetPos){
    // Updates the position of the target
}


float LocomotionPlanning::Stop::evaluate(double player_model){
    return 0.0;
}
