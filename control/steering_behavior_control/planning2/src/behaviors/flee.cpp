#include "planning2/behaviors/flee.h"

geometry_msgs::Vector3 LocomotionPlanning::Flee::calculate_desired_velocity(geometry_msgs::Point32 current_pos){
    geometry_msgs::Vector3 output;
    output = VectorUtility::vector_difference(target_, current_pos);
    output = VectorUtility::normalize(output);
    output = VectorUtility::scalar_multiply(output, max_speed_);
    return output;
}

geometry_msgs::Vector3 LocomotionPlanning::Flee::calculate_steering_force(geometry_msgs::Vector3 current_vel, geometry_msgs::Vector3 desired_vel){
    geometry_msgs::Vector3 output;
    output = VectorUtility::vector_difference(current_vel, desired_vel);
    output = VectorUtility::scalar_multiply(output, max_speed_);
    return output;
}

float LocomotionPlanning::Flee::evaluate(double player_model){
    return 0.0;
}

void LocomotionPlanning::Flee::updateTargetPos(geometry_msgs::Point32 newTargetPos){
    
}

bool LocomotionPlanning::Flee::targetReached(geometry_msgs::Point32 currentPos){
    return false;
}


// bool LocomotionPlanning::Flee::updateTarget(sensor_msgs::LaserScan scan, geometry_msgs::Point32 current_pos, float current_rotation_wrt_map){
//     float min_dist = 6.0; // minimum distance from sensor is 5.6 meters
//     int min_index;

//     for(int i=0; i<1000; i++){
//         if(scan.ranges[i] < min_dist){
//             min_dist = scan.ranges[i];
//             min_index = i;
//         }
//     }

//     float obstacle_angle = min_index * (2*M_PI/1000);

//     geometry_msgs::Point32 target;
//     target.x = current_pos.x + cos(obstacle_angle + current_rotation_wrt_map + M_PI)*min_dist;
//     target.y = current_pos.y + sin(obstacle_angle + current_rotation_wrt_map + M_PI)*min_dist;

//     setTarget(target);

//     if(to_update_){
//         to_update_ = false;
//         return true;
//     }

//     return false;
// }



// std::vector<float> LocomotionPlanning::Flee::getUpdateWeights(){
//     std::vector<float> output;

// 	output.resize(3);
// 	output[0] = 0.4;
// 	output[1] = 1.1;
// 	output[2] = 2.0;
			
//     return output;
// }
