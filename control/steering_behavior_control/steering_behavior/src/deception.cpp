#include "steering_behavior/deception.h"
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3.h>

geometry_msgs::Vector3 SteeringBehavior::Deception::calculate_desired_velocity(geometry_msgs::Point32 current_pos){
    geometry_msgs::Vector3 output;
    
    // ROLLBACK TO SEEK /////////////////////////////////////////////////
    // output = VectorUtility::vector_difference(current_pos, target_);
    // output = VectorUtility::normalize(output);
    // output = VectorUtility::scalar_multiply(output, max_speed_);
    //////////////////////////////////////////////////////////////////////

    // ROLLBACK TO ARRIVAL ///////////////////////////////////////////////
    float distance_from_target;
    float ramped_speed;
    float clipped_speed;
    distance_from_target = VectorUtility::distance(target_, current_pos);

    ramped_speed = max_speed_ * (distance_from_target / slowing_radius_);
    clipped_speed = std::min(ramped_speed, max_speed_);
    
    output = VectorUtility::vector_difference(current_pos, target_);
    output = VectorUtility::scalar_multiply(output, (clipped_speed / distance_from_target));
    //////////////////////////////////////////////////////////////////////

    return output;
}

geometry_msgs::Vector3 SteeringBehavior::Deception::calculate_steering_force(geometry_msgs::Vector3 current_vel, geometry_msgs::Vector3 desired_vel){
    geometry_msgs::Vector3 output;
    output = VectorUtility::vector_difference(current_vel, desired_vel);
    return output;
}

float SteeringBehavior::Deception::evaluate(double player_model){
    return 1;
}

bool SteeringBehavior::Deception::updateTarget(sensor_msgs::LaserScan scan, geometry_msgs::Point32 current_pos, float current_rotation_wrt_map){
    if(current_target_index_ == targets_.size() - 1){
        return false;
    }

    float distance_from_target = VectorUtility::distance(current_pos, target_);

    if(distance_from_target < deception_target_change_distance_){
        // We have to change target to the real one
        current_target_index_ = current_target_index_ + 1;
        target_ = targets_[current_target_index_];
        return true;
    }
    return false;
}

void SteeringBehavior::Deception::updateTargetPos(std::vector<geometry_msgs::Point32> towers){
    if(current_target_index_ == targets_.size() - 1){
        // We're heading to real target
        target_ = towers[tower_target_index_];
    }
}

std::vector<float> SteeringBehavior::Deception::getUpdateWeights(){
            std::vector<float> output;
            output.resize(3);
            if(targets_.size() == 2){
                // Mass reduction factor
                output[0] = 0.7;
                // Speed increase factor
                output[1] = 1.15;
                // Force increase factor
                output[2] = 2.0;
            }
            else{
                // Mass reduction factor
                output[0] = 0.8;
                // Speed increase factor
                output[1] = 1.2;
                // Force increase factor
                output[2] = 1.1;
            }
            return output;
        }