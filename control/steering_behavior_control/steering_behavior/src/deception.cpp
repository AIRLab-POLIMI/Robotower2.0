#include "steering_behavior/deception.h"
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>

geometry_msgs::Vector3 SteeringBehavior::Deception::calculate_desired_velocity(geometry_msgs::Point32 current_pos){
    geometry_msgs::Vector3 output;
    output = VectorUtility::vector_difference(current_pos, target_);
    return output;
}

geometry_msgs::Vector3 SteeringBehavior::Deception::calculate_steering_force(geometry_msgs::Vector3 current_vel, geometry_msgs::Vector3 desired_vel){
    geometry_msgs::Vector3 output;
    
    output = VectorUtility::vector_difference(current_vel, desired_vel);
    return output;
}

float SteeringBehavior::Deception::evaluate(){
    return 1;
}

bool SteeringBehavior::Deception::updateTarget(sensor_msgs::LaserScan scan, geometry_msgs::Point32 current_pos, float current_rotation_wrt_map){
    ROS_ERROR("REACHING TARGET x:%.2f y:%.2f", target_.x, target_.y);
    if(current_target_index_ == targets_.size() - 1){
        // We're going towards the real target
        ROS_INFO("Heading to real target");
        return false;
    }

    float distance_from_target = VectorUtility::distance(current_pos, target_);

    if(distance_from_target < 1){
        ROS_INFO("Changing target!");
        // We have to change target to the real one
        current_target_index_ = current_target_index_ + 1;
        target_ = targets_[current_target_index_];
        return true;
    }
    return false;
}

std::vector<float> SteeringBehavior::Deception::getUpdateWeights(){
            std::vector<float> output;
            output.resize(3);
            if(targets_.size() == 2){
                output[0] = 2.0;
                output[1] = 2.0;
                output[2] = 2.0;
            }
            else{
                output[0] = 1.5;
                output[1] = 1.4;
                output[2] = 1.3;
            }
            return output;
        }