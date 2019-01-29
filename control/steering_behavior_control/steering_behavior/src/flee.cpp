#include "steering_behavior/flee.h"
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3.h>

geometry_msgs::Vector3 SteeringBehavior::Flee::calculate_desired_velocity(geometry_msgs::Point32 current_pos){
    geometry_msgs::Vector3 output;
    output = VectorUtility::vector_difference(target_, current_pos);
    output = VectorUtility::normalize(output);
    output = VectorUtility::scalar_multiply(output, max_speed_);
    return output;
}

geometry_msgs::Vector3 SteeringBehavior::Flee::calculate_steering_force(geometry_msgs::Vector3 current_vel, geometry_msgs::Vector3 desired_vel){
    geometry_msgs::Vector3 output;
    output = VectorUtility::vector_difference(current_vel, desired_vel);
    output = VectorUtility::scalar_multiply(output, max_speed_);
    return output;
}

bool SteeringBehavior::Flee::evaluateSafety(geometry_msgs::Point32 current_pos){
    float distance_from_target = VectorUtility::distance(current_pos, target_);
    if(distance_from_target > safety_distance_){
        return true;
    }
    return false;
}

float SteeringBehavior::Flee::evaluate(double player_model){
    return 0.0;
}

bool SteeringBehavior::Flee::updateTarget(sensor_msgs::LaserScan scan, geometry_msgs::Point32 current_pos, float current_rotation_wrt_map){
    float min_dist = 6.0; // minimum distance from sensor is 5.6 meters
    int min_index;
    geometry_msgs::Point32 target;

    tf::StampedTransform robotCenterTransform_;
    if(to_update_){
        change_randomized_center_index_ = true; // Change it first time
    }
    try{
        listener_.waitForTransform("/base_link", ros::Time(0), "/playground_center", ros::Time(0), "/map", ros::Duration(0.10));
        listener_.lookupTransform("/base_link", "/playground_center", ros::Time(0), robotCenterTransform_);
       
        float x_center = robotCenterTransform_.getOrigin().x();
        float y_center = robotCenterTransform_.getOrigin().y();

        float alpha_center = atan2(y_center, x_center);
        float proportion = ((alpha_center + M_PI) / (2*M_PI));
        int center_scan_index = proportion * scan.ranges.size();

        std::random_device rd;
        std::mt19937 eng(rd());
        std::uniform_int_distribution<> distr(center_scan_index-30, center_scan_index+30);

        if(change_randomized_center_index_){
            change_randomized_center_index_ = false;
            randomized_center_index_ = distr(eng);
        }

        // ROS_WARN("Current rotation wrt map %.2f", current_rotation_wrt_map);
        // ROS_WARN("Center angle wrt map %.2f", alpha_center*(180/M_PI));
        // alpha_center = alpha_center + current_rotation_wrt_map;
        // ROS_WARN("Center angle with rotation %.2f", alpha_center*(180/M_PI));
        // ROS_WARN("Center playground index: %d", center_scan_index);

        int next_index = randomized_center_index_;
        int previous_index = randomized_center_index_;
        bool is_clear = true;


        for(int i=0; i<60; i++){
            if(scan.ranges[next_index] < 1 || scan.ranges[previous_index] < 1){
                is_clear = false;
            }
            next_index = (next_index + 1 >= scan.ranges.size()) ? 0: (next_index + 1);
            previous_index = (previous_index - 1 < 0) ? (scan.ranges.size() - 1): (previous_index - 1);
        }

        float obstacle_angle;

        if(is_clear){
            change_randomized_center_index_ = false; // Keep it until it's clear to go
            obstacle_angle = randomized_center_index_ * ((2*M_PI)/1000);
            target.x = current_pos.x + cos(obstacle_angle + current_rotation_wrt_map)*1;
            target.y = current_pos.y + sin(obstacle_angle + current_rotation_wrt_map)*1;
        }
        else{
            change_randomized_center_index_ = true; // Change it when the way is obstructed
            for(int i=0; i<scan.ranges.size(); i++){
                if(scan.ranges[i] < min_dist){
                    min_dist = scan.ranges[i];
                    min_index = i;
                }
            }
            obstacle_angle = min_index * (2*M_PI/1000);
            target.x = current_pos.x + cos(obstacle_angle + current_rotation_wrt_map + M_PI)*min_dist;
            target.y = current_pos.y + sin(obstacle_angle + current_rotation_wrt_map + M_PI)*min_dist;
        }

    } catch (const std::exception& ex){
        ROS_WARN_STREAM(ex.what());
        for(int i=0; i<scan.ranges.size(); i++){
            if(scan.ranges[i] < min_dist){
                min_dist = scan.ranges[i];
                min_index = i;
            }
        }
        float obstacle_angle = min_index * (2*M_PI/1000);
        target.x = current_pos.x + cos(obstacle_angle + current_rotation_wrt_map + M_PI)*min_dist;
        target.y = current_pos.y + sin(obstacle_angle + current_rotation_wrt_map + M_PI)*min_dist;
    }

    setTarget(target);

    if(to_update_){
        to_update_ = false;
        return true;
    }

    return false;
}



std::vector<float> SteeringBehavior::Flee::getUpdateWeights(){
    std::vector<float> output;

	output.resize(3);
	output[0] = 0.4;
	output[1] = 1.1;
	output[2] = 2.0;
			
    return output;
}


