#include "steering_behavior/arrival.h"
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>

geometry_msgs::Vector3 SteeringBehavior::Arrival::calculate_desired_velocity(geometry_msgs::Point32 current_pos){
    geometry_msgs::Vector3 output;
    float distance_from_target;
    float ramped_speed;
    float clipped_speed;
    distance_from_target = VectorUtility::distance(target_, current_pos);

    ramped_speed = max_speed_ * (distance_from_target / slowing_radius_);
    clipped_speed = std::min(ramped_speed, max_speed_);
    ROS_ERROR("Clipped speed: %.2f", clipped_speed);
    output = VectorUtility::vector_difference(current_pos, target_);
    output = VectorUtility::scalar_multiply(output, (clipped_speed / distance_from_target));

    // if(distance_from_target < slowing_radius_){
    //     ROS_ERROR("SLOWING DOWN");
    //     float desired_speed = distance_from_target / slowing_radius_;
    //     ROS_ERROR("Speed from target: %.2f", desired_speed);
        
    //     ROS_INFO("Original vel x:%.2f, y:%.2f, z:%.2f", output.x, output.y, output.z);

    //     output = VectorUtility::normalize(output);
    //     output = VectorUtility::scalar_multiply(output, desired_speed); 
    //     // output = VectorUtility::revert(output);       
    //     ROS_INFO("Slowed down vel x:%.2f, y:%.2f, z:%.2f", output.x, output.y, output.z);

    // }
    // else{
    //     output = VectorUtility::normalize(output);
    //     output = VectorUtility::scalar_multiply(output, 1); 
    // }
    return output;
}

geometry_msgs::Vector3 SteeringBehavior::Arrival::calculate_steering_force(geometry_msgs::Vector3 current_vel, geometry_msgs::Vector3 desired_vel){
    geometry_msgs::Vector3 output;
    
    output = VectorUtility::vector_difference(current_vel, desired_vel);
    return output;
}