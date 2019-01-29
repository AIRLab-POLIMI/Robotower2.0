#include <ros/ros.h>
#include "planning/safety_evaluator.h"
#include "planning/SafetyMsg.h"
#include <std_msgs/Bool.h>

#define SPEED_THD 0.1
#define COLLIDING_ANGLE_THD 0.785

Safety::SafetyEvaluator::SafetyEvaluator(){
    if (!nh_.getParam("/planning/vel_topic_sub", vel_topic_)){
            ROS_ERROR("SAFETY PLANNER: could not read 'vel_topic_sub' from rosparam!");
            vel_topic_ = "/vel";
            // exit(-1);
    }

    if (!nh_.getParam("/planning/player_angle_topic", player_angle_topic_)){
        ROS_ERROR("SAFETY PLANNER: could not read 'player_angle_topic' from rosparam!");
        player_angle_topic_ = "/player_filtered";
        // exit(-1);
    }

    if (!nh_.getParam("/planning/safety_treshold", safety_treshold_)){
        ROS_ERROR("SAFETY PLANNER: could not read 'safety_treshold' from rosparam!");
        safety_treshold_ = 1.0;
        // exit(-1);
    }

    if (!nh_.getParam("/planning/safety_topic", safety_topic_)){
        ROS_ERROR("SAFETY MANAGER: could not read 'safety_topic' from rosparam!");
        safety_topic_ = "/safety";
        // exit(-1);
    }

    if (!nh_.getParam("/planning/safety_treshold_flee", safety_treshold_flee_)){
        ROS_ERROR("SAFETY MANAGER: could not read 'safety_topic' from rosparam!");
        safety_treshold_flee_ = 0.7;
        // exit(-1);
    }

	if (!nh_.getParam("/planning/collision_angle_treshold", collision_angle_treshold_)){
        ROS_ERROR("SAFETY MANAGER: could not read 'collision_angle_treshold' from rosparam!");
        collision_angle_treshold_ = 1.57079632;
        // exit(-1);
    }

    if (!nh_.getParam("/planning/window_width", window_width_)){
        ROS_ERROR("SAFETY MANAGER: could not read 'window_width' from rosparam!");
        window_width_ = 1.57079632;
        // exit(-1);
    }

    if (!nh_.getParam("/planning/speed_thd", speed_thd_)){
        ROS_ERROR("SAFETY MANAGER: could not read 'window_width' from rosparam!");
        speed_thd_ = 0.1;
        // exit(-1);
    }

    // ROLLBACK
    laser_sub_ = nh_.subscribe("/scan", 1, &Safety::SafetyEvaluator::laserCallback, this);
    // laser_sub_ = nh_.subscribe("/scan_player_tracking", 1, &Safety::SafetyEvaluator::laserCallback, this);
    player_angle_sub_ = nh_.subscribe(player_angle_topic_, 1, &Safety::SafetyEvaluator::playerCallback, this);
    vel_sub_ = nh_.subscribe(vel_topic_, 1, &Safety::SafetyEvaluator::velCallback, this);
    safety_pub_ = nh_.advertise<planning::SafetyMsg>(safety_topic_, 1);

    section_angular_width_ = 360 / number_of_sections_;
    currently_safe_ = true;

    count_ = 0;
    other_count_ = 0;

    current_heading_sector_ = 0;
    current_player_sector_ = 1;
}

// bool Safety::SafetyEvaluator::isCollidingDirection(){
//     // if(current_heading_sector_ == current_player_sector_){
//     //     return true;
//     // }
    
//     // return false;
//     int next_sector  = current_player_sector_ + 1;
//     if(next_sector == number_of_sections_){
//         next_sector = 0;
//     }
//     int previous_sector  = current_player_sector_ - 1;
//     if(previous_sector == -1){
//         previous_sector = number_of_sections_ - 1;
//     }


//     if(current_heading_sector_ == next_sector){
//         return true;
//     }
//     else if(current_heading_sector_ == previous_sector){
//         return true;
//     }
//     else if(current_heading_sector_ == current_player_sector_){
//         return true;
//     }
//     return false;
// }

bool Safety::SafetyEvaluator::isCollidingDirection(){
    // Function to evaluate wheter we are moving in the direction of the player or not, given a tolerance treshold
    float angle_diff = calculateAngleDiff(current_angle_motion_, current_angle_player_);
    // ROS_INFO("ANGLE DIFF: %.2f", angle_diff);
	
	// Divided by two because the angle diff is absolute
	// Checking if player is in [angle_motion - COLLIDING_ANGLE_THD/2, angle_motion + COLLIDING_ANGLE_THD/2]
    if( angle_diff < collision_angle_treshold_/2){	
        // ROS_ERROR("ANGLE DIFF: %.2f", angle_diff);
        // ROS_ERROR("WE'RE MOVING TOWARDS PLAYER");
        return true;
    }
    return false;
}

// bool Safety::SafetyEvaluator::isPlayerCloseToTower(){

// }

float Safety::SafetyEvaluator::calculateAngleDiff(float angle1, float angle2){
    float abs_diff;
    // Transforming angles to (0, 2 pi) interval
    float angle_tmp_1 = angle1 + M_PI;
    float angle_tmp_2 = angle2 + M_PI;
    abs_diff = fabs(angle_tmp_1 - angle_tmp_2);
    if(abs_diff > M_PI){
        return (2 * M_PI) - abs_diff;
    }
    return abs_diff;
}

bool Safety::SafetyEvaluator::checkAllScan(){
    // Check the whole scan if there's something below treshold
    float min_dist = 6.0;
    for(int i=0; i<current_scan_.ranges.size(); i++){
        if(current_scan_.ranges[i] < min_dist){
            min_dist = current_scan_.ranges[i];
        }
    }

    if(min_dist <= safety_treshold_flee_){
        // We are in a non safe condition
        // ROS_WARN("UNSAFE");
        return false;
    }
    else{
        return true;
    }
}

bool Safety::SafetyEvaluator::checkSector(int idx){
    // Checks if there's something too close in the direction of movement
    std::vector<float> sector_ranges;
    sector_ranges = current_scan_segmented_[idx];

    float min_dist = 6.0;
    int min_index = 0;
    int section_lenght = current_scan_.ranges.size() / number_of_sections_;

    for(int i=0; i<section_lenght; i++){
        if(sector_ranges[i] < min_dist){
            min_dist = sector_ranges[i];
            min_index = i;
        }
    }
    // ROS_INFO("Min dist: %.2f", min_dist);
    if(min_dist <= safety_treshold_){
        // We are in a non safe condition
        // ROS_WARN("UNSAFE");
        return false;
    }
    else{
        return true;
    }
}

bool Safety::SafetyEvaluator::evaluateSafety(){
    // Function to evaluate whether the navigation is safe or not

    // Check whether the player is in the motion direction
    if(current_speed_ > speed_thd_){
        // ROS_ERROR("Speed: %.2f", current_speed_);
        // DO NOT TAKE ACTION IF SPEED TOO LOW
        if(!currently_safe_){
            currently_safe_ = checkAllScan();
        }
        else{
            // ROS_WARN("Checking movement direction");
            
            // ROLLBACK uncomment the condition evaluation
            bool is_colliding_direction = isCollidingDirection();
            // bool is_colliding_direction = true;
            if(is_colliding_direction){
                // Identify the one we're moving towards
                // Check if there's something close to it
                // If so, we're NOT safe
                // Else we're safe
                // currently_safe_ = checkHeadingSector();
                currently_safe_ = isHeadingSectorSafe();
            }
        }
    }

    safety_msg_.safety = currently_safe_;
    safety_msg_.section_index = current_heading_sector_;

    safety_pub_.publish(safety_msg_);
    return currently_safe_;
}

void Safety::SafetyEvaluator::laserCallback(const sensor_msgs::LaserScan& scan){
    current_scan_ = scan;
}

bool Safety::SafetyEvaluator::isHeadingSectorSafe(){
    // Function to check wheter the sector where the robot is heading is free from the player

	int heading_idx = convertAngleToIndex(current_angle_motion_); // Index where the robot is virtually headed to

    int number_of_indexes = getNumberOfScans(window_width_); // Sector dimension given in number of scan indexes

    // ROS_INFO("Number of indexes: %d", number_of_indexes);

    // Now we have to use the index where the robot is virtually headed to as center of heading sector
	// We should start from half width before the center
	int starting_index = heading_idx - (number_of_indexes/2);
	starting_index = validateNegativeIndex(starting_index);

    return isWindowSafe(starting_index, number_of_indexes);
}

// CHANGE HEADER FILE
bool Safety::SafetyEvaluator::isWindowSafe(int starting_index, int number_of_indexes){
	// Function to check that safety condition is met in the analyzed window
	// Iterates through the laser scan given starting position an number of steps
	
	int idx_to_check = starting_index;
    // ROS_INFO("Checking from %d", idx_to_check);
    for(int i=0; i<number_of_indexes; i++){
        idx_to_check = getNextIndex(idx_to_check);
        if(current_scan_.ranges[idx_to_check] < safety_treshold_){
			// If we find something below the safety treshold
            return false;
        }
    }
    // ROS_INFO("TO %d", idx_to_check);
    return true;
}
// CHANGE HEADER FILE
int Safety::SafetyEvaluator::validateNegativeIndex(int idx){
	if(idx < 0){
        idx = current_scan_.ranges.size() + idx;
    }
	return idx;
}
// CHANGE HEADER FILE
int Safety::SafetyEvaluator::getNextIndex(int idx){
	// Function to return the next index taking care of circularity of vector
	int out_idx = idx + 1;
	if(out_idx == current_scan_.ranges.size()){
		return 0;
	}
	return out_idx;
}
// CHANGE HEADER FILE
int Safety::SafetyEvaluator::convertAngleToIndex(float theta){
	// Function to convert an angle in (-pi, pi) to the corresponding index of the scan
	// First we translate the domain of the angle to (0, 2pi)
	float theta_trans = theta + M_PI;

	// Then we evaluate the proportion theta_trans:2pi = idx:scan_size
	int idx;
	idx = (theta_trans / (2*M_PI)) * current_scan_.ranges.size();
	return idx;
}
// CHANGE HEADER FILE
int Safety::SafetyEvaluator::getNumberOfScans(float theta){
	// Function to convert an angle in (-pi, pi) to the corresponding number of scan indexes
	// First we get the dimension on angle increment due to one scan index in radians
	float angle_increment = (2*M_PI) / current_scan_.ranges.size();

	// Then we evaluate the number of indexes
	int number_of_indexes;
	number_of_indexes = theta / angle_increment;
	return number_of_indexes;
}

// CHANGE HEADER FILE
float Safety::SafetyEvaluator::convertIndexToAngle(int idx){
	// Function to convert an index of the scan to the corresponding angle in (-pi, pi)
	// First we evaluate the proportion theta_trans:2pi = idx:scan_size
	float theta_trans; // Angle in (0,2pi)
	theta_trans = idx * (2*M_PI / current_scan_.ranges.size());

	// Then we translate the domain of the angle to (-pi,+pi)
	float theta = theta_trans - M_PI;
	return theta;
}

void Safety::SafetyEvaluator::velCallback(const geometry_msgs::Twist& vel){
    current_speed_ = sqrt(pow(vel.linear.y,2) + pow(vel.linear.x,2));
    current_angle_motion_ = atan2(vel.linear.y, vel.linear.x);
    // int current_angle_deg = current_angle_motion_ * (180 / M_PI); // Converting to degrees
    // current_heading_sector_ = (current_angle_deg + 180) / section_angular_width_;
    // ROS_WARN("Current motion angle: %.2f", current_angle_motion_ * (180/M_PI));
    // ROS_WARN("Current heading sector: %d", current_heading_sector_);
}

void Safety::SafetyEvaluator::playerCallback(const geometry_msgs::PointStamped& position){
    player_distance_ = sqrt(pow(position.point.y,2) + pow(position.point.x,2));
    current_angle_player_ = atan2(position.point.y, position.point.x);
    // int current_angle_deg = current_angle_player_ * (180 / M_PI);  // Converting to degrees
    // current_player_sector_ = (current_angle_deg + 180) / section_angular_width_;
    // ROS_WARN("Current player angle: %.2f", current_angle_player_ * (180/M_PI));
    // ROS_WARN("Current player sector: %d", current_player_sector_);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "planning_safety_node");
    ros::NodeHandle nh;
    ros::Rate rate(30);

    Safety::SafetyEvaluator safety_evaluator;

    while(ros::ok()){
        safety_evaluator.evaluateSafety();
        ros::spinOnce();
        rate.sleep();
    }
}