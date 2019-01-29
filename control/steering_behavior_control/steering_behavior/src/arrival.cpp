#include "steering_behavior/arrival.h"
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3.h>

geometry_msgs::Vector3 SteeringBehavior::Arrival::calculate_desired_velocity(geometry_msgs::Point32 current_pos){
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

geometry_msgs::Vector3 SteeringBehavior::Arrival::calculate_steering_force(geometry_msgs::Vector3 current_vel, geometry_msgs::Vector3 desired_vel){
    geometry_msgs::Vector3 output;
    
    output = VectorUtility::vector_difference(current_vel, desired_vel);
    return output;
}

float SteeringBehavior::Arrival::evaluate(double player_model){
    return 4;
}