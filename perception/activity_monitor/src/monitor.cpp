#include "activity_monitor/activity_monitor.h"

#include <ros/ros.h>


#define STATIONARY_TRESHOLD 0.4 // displacement of under 40cm is considered stationary
#define MOTION_DIRECTION_TRESHOLD 0.785398  // Variation of 45 deg in motion direction is considered same target
#define NUM_TOWERS 4
#define CLAMPING_TERM 1 // Term to avoid infinite value when considering the inverse 

ActivityMonitoring::ActivityMonitor::ActivityMonitor(){
    current_player_tower_distance_.resize(NUM_TOWERS);
    nh_.subscribe("/player_filtered", 1, &ActivityMonitoring::ActivityMonitor::playerPositionCallback, this);
}

void ActivityMonitoring::ActivityMonitor::playerPositionCallback(const geometry_msgs::PointStamped position){
    geometry_msgs::Point evidence_wrt_map = transformPointToMap(position.point);
    updateDistances(evidence_wrt_map);
    if(isStationary(evidence_wrt_map)){
        // Player is not moving or reacting
    }
    else{
        // Player is moving
        evaluateMotionWrtRobot(evidence_wrt_map);
        if(is_monitoring_target_){
            int motion_wrt_target = evaluateMotionWrtTarget(evidence_wrt_map);
            if(motion_wrt_target == 1){
                // Player has started to go towards the tower
                // estimatePlayerHyperparameter();
                // is_monitoring_target_ = false;
                // updateModel();
            }
        }
    }

    current_player_position_ = evidence_wrt_map;
}

bool ActivityMonitoring::ActivityMonitor::isStationary(geometry_msgs::Point new_pos){
    double displacement = sqrt(pow(new_pos.x - current_player_position_.x, 2) + pow(new_pos.y - current_player_position_.y, 2));
    if(displacement < STATIONARY_TRESHOLD){
        return false;
    }
    return true;
}

geometry_msgs::Point ActivityMonitoring::ActivityMonitor::transformPointToMap(geometry_msgs::Point in){
    geometry_msgs::Point out;

    updateCurrentRobotPos();

    double angle_wrt_robot = atan2(in.y, in.x) /*- M_PI*/;
    double distance = sqrt(pow(in.x, 2) + pow(in.y, 2));
    out.x = cos(angle_wrt_robot + current_robot_rotation_wrt_map_) * distance;
    out.y = sin(angle_wrt_robot + current_robot_rotation_wrt_map_) * distance;

    return out;
}

int ActivityMonitoring::ActivityMonitor::evaluateMotionWrtRobot(geometry_msgs::Point new_pos){
    double robot_direction = atan2(current_player_position_.y - current_robot_pos_.y, current_player_position_.x - current_robot_pos_.x);
    
    double motion_direction = atan2(current_player_position_.y - new_pos.y, current_player_position_.x - new_pos.x);

    // Translate both from [-pi, pi] to [0, 2pi] to ease calculations
    robot_direction = robot_direction + M_PI;
    motion_direction = motion_direction + M_PI;

    double direction_diff = fabs(robot_direction - motion_direction);
    if(direction_diff < MOTION_DIRECTION_TRESHOLD){
        return 1;
    }
    else{
        return -1;
    }
}

int ActivityMonitoring::ActivityMonitor::evaluateMotionWrtTarget(geometry_msgs::Point new_pos){
    double target_direction = atan2(current_player_position_.y - tower_pos_[current_tower_target_].y, current_player_position_.x - tower_pos_[current_tower_target_].x);
    
    double motion_direction = atan2(current_player_position_.y - new_pos.y, current_player_position_.x - new_pos.x);

    // Translate both from [-pi, pi] to [0, 2pi] to ease calculations
    target_direction = target_direction + M_PI;
    motion_direction = motion_direction + M_PI;

    double direction_diff = fabs(target_direction - motion_direction);
    if(direction_diff < MOTION_DIRECTION_TRESHOLD){
        return 1;
    }
    else{
        return -1;
    }
}

void ActivityMonitoring::ActivityMonitor::monitorTargetCallback(const std_msgs::Int8 target_index){
    current_tower_target_ = target_index.data;
    is_monitoring_target_ = true;
}

void ActivityMonitoring::ActivityMonitor::abortMonitorTargetCallback(const std_msgs::Int8 target_index){
    is_monitoring_target_ = false;
}

void ActivityMonitoring::ActivityMonitor::updateDistances(geometry_msgs::Point new_pos){
    for(int i=0; i<NUM_TOWERS; i++){
        current_player_tower_distance_[i] = sqrt(pow(tower_pos_[i].x - new_pos.x, 2) + pow(tower_pos_[i].y - new_pos.y, 2));
        if(current_player_tower_distance_[i] < current_player_tower_distance_[closest_tower_index_]){
            closest_tower_index_ = i;
        }
        current_robot_tower_distance_[i] = sqrt(pow(tower_pos_[i].x - current_robot_pos_.x, 2) + pow(tower_pos_[i].y - current_robot_pos_.y, 2));
        current_robot_tower_angles_[i] = atan2(current_robot_pos_.y - tower_pos_[i].y, current_robot_pos_.x - tower_pos_[i].x);
    }
    current_robot_player_distance_ = sqrt(pow(current_robot_pos_.x - new_pos.x, 2) + pow(current_robot_pos_.y - new_pos.y, 2));
}

double ActivityMonitoring::ActivityMonitor::calculateTowerAttraction(int tower_index){
    return 1.0 / (current_player_tower_distance_[tower_index] + CLAMPING_TERM);
}

double ActivityMonitoring::ActivityMonitor::calculateRobotPerturbation(int tower_index){
    double projection = current_robot_speed_ * cos(current_robot_tower_angles_[tower_index]);
    double tower_danger = 1.0 / (current_robot_tower_distance_[tower_index] + CLAMPING_TERM);

    return projection + tower_danger;
}

double ActivityMonitoring::ActivityMonitor::estimatePlayerHyperparameter(){
    double alpha_diff = calculateTowerAttraction(current_tower_target_) - calculateTowerAttraction(closest_tower_index_);
    double beta_diff = calculateRobotPerturbation(closest_tower_index_) - calculateRobotPerturbation(current_tower_target_);

    return beta_diff / alpha_diff;
}



void ActivityMonitoring::ActivityMonitor::velCallback(const geometry_msgs::Twist vel){
    current_robot_speed_ = sqrt(pow(current_robot_vel_.x, 2) + pow(current_robot_vel_.y, 2));
    current_robot_vel_ = rotateToMap(vel.linear);
}


void ActivityMonitoring::ActivityMonitor::updateCurrentRobotPos(){
       
        tf::StampedTransform transform;
        try{
            listener_.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(10.0));
            listener_.lookupTransform("/map", "/base_link", ros::Time(0), transform);
            current_robot_pos_.x = transform.getOrigin().getX();
            current_robot_pos_.y = transform.getOrigin().getY();
            current_robot_pos_.z = transform.getOrigin().getZ();


            double roll, pitch, yaw;
            tf::Quaternion quat = transform.getRotation();            
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            
            current_robot_rotation_wrt_map_ = yaw;
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
}

geometry_msgs::Vector3 ActivityMonitoring::ActivityMonitor::rotateToMap(geometry_msgs::Vector3 vector){
    geometry_msgs::Vector3 output;

    double current_orientation = atan2(vector.y, vector.x);
    double magnitude = current_robot_speed_;
    double new_orientation = current_orientation + current_robot_rotation_wrt_map_;

    output.x = magnitude * cos(new_orientation);
    output.y = magnitude * sin(new_orientation);
    output.z = 0.0;
    return output;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "activity_monitor_node");
    ros::NodeHandle nh;
    ros::Rate rate(1);

   ActivityMonitoring::ActivityMonitor monitor;

    while(ros::ok()){
        ROS_INFO("WELLA");
        ros::spinOnce();
        rate.sleep();
    }
}