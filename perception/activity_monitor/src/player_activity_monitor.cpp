#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "activity_monitor/player_activity_monitor.h"

#include <string.h>

#define STATIONARY_TRESHOLD 0.05 // displacement of under 20cm is considered stationary
#define MOTION_DIRECTION_TRESHOLD 0.785398  // Variation of 45 deg in motion direction is considered same target
#define VEL_TRESHOLD 0.1
#define NUM_TOWERS 4


PlayerActivityMonitoring::PlayerActivityMonitor::PlayerActivityMonitor(){
    current_player_tower_distance_.resize(NUM_TOWERS, 0.0);
    current_robot_tower_distance_.resize(NUM_TOWERS, 0.0);
    current_player_tower_distance_variation_.resize(NUM_TOWERS, 0.0);
    current_robot_tower_distance_variation_.resize(NUM_TOWERS, 0.0);
    robot_velocity_projections_.resize(NUM_TOWERS, 0.0);

    tower_pos_.resize(4);
    std::vector<float> tower;
    for(int i=0; i<4; i++){
        std::string param_name;
        param_name = "/tower_" + std::to_string(i+1);
        if (!nh_.getParam(param_name, tower)){
            ROS_ERROR("STEERING BEHAVIOR MANAGER: could not read 'tower' from rosparam!");
            exit(-1);
        }
        geometry_msgs::Point32 pos;
        pos.x = tower[0];
        pos.y = tower[1];
        tower_pos_[i] = pos;
    }

    player_pos_sub_ = nh_.subscribe("/player_filtered", 1, &PlayerActivityMonitoring::PlayerActivityMonitor::playerPositionCallback, this);

    vel_sub_ = nh_.subscribe("/vel", 1, &PlayerActivityMonitoring::PlayerActivityMonitor::velCallback, this);
    tower_pos_sub_ = nh_.subscribe("/tower_pos", 1, &PlayerActivityMonitoring::PlayerActivityMonitor::towerPosCallback, this);
    activity_pub_ = nh_.advertise<activity_monitor::Activity>("/player_activity", 1);
    player_target_pub_ = nh_.advertise<visualization_msgs::Marker>("/player_target", 1);
    risk_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/risk_tower", 1);
}


void PlayerActivityMonitoring::PlayerActivityMonitor::playerPositionCallback(const geometry_msgs::PointStamped position){
    /* 
    When receiving the player position we take a snapshot of the current configuration of the game and publish it.
    We refer every position in the map frame.
    */

    // Player position in the map frame
    geometry_msgs::Point evidence_wrt_map = transformPointToMap(position);
    
    
    // Update distance between relevant positions in the game
    updateDistances(evidence_wrt_map);
    updateVelocityProjections();
    publishRiskyTower();

    // Detected player activity
    if(isStationary(evidence_wrt_map)){
        // Player is not moving
        // ROS_WARN("STATIONARY!!!!!!!!!");
        activity_msg_.is_stationary = true;
        // activity_msg_.target_tower = getClosestTowerIndex();
    }
    else{
        // Player is moving
        // ROS_INFO("MOVING!");
        activity_msg_.is_stationary = false;
        // Evaluate the tower he's heading to
        activity_msg_.target_tower = evaluatePlayerTargetTower(evidence_wrt_map);
        publishPlayerTargetMarker();
    }
    // Find the closest tower to the player
    activity_msg_.closest_tower_index = getClosestTowerIndex();
    // Update player's position
    current_player_position_ = evidence_wrt_map;

    // Publish state of the game
    publishActivityMsg();
}

void PlayerActivityMonitoring::PlayerActivityMonitor::updateDistances(geometry_msgs::Point new_pos){
    int i;

    for(i=0; i < NUM_TOWERS; i++){
        // Calculate current distance of player and robot from towers
        double distance_player_tower = sqrt(pow(tower_pos_[i].x - new_pos.x, 2.0) + pow(tower_pos_[i].y - new_pos.y, 2.0));
        double distance_robot_tower = sqrt( pow( (tower_pos_[i].x - current_robot_pos_.x), 2.0 ) + pow( (tower_pos_[i].y - current_robot_pos_.y), 2.0) );

        // ROS_INFO("Tower %d distance from robot: %.3f", i, distance_robot_tower);
        // ROS_INFO("Tower %d pos x:%.2f, y:%.2f / Robot pos x:%.2f y:%.2f", i, tower_pos_[i].x, tower_pos_[i].y, current_robot_pos_.x, current_robot_pos_.y);
        // ROS_INFO("Tower %d distance from player: %.3f", i, distance_player_tower);
        
        // Calculate variation of distance of player and robot from towers from previous observation
        current_player_tower_distance_variation_[i] = distance_player_tower - current_player_tower_distance_[i];
        current_robot_tower_distance_variation_[i] =  distance_robot_tower - current_robot_tower_distance_[i];

        // Update observations
        current_player_tower_distance_[i] = distance_player_tower;
        current_robot_tower_distance_[i] = distance_robot_tower;
    }
    
    double player_robot_distance = sqrt(pow(current_robot_pos_.x - new_pos.x, 2) + pow(current_robot_pos_.y - new_pos.y, 2));
    current_robot_player_distance_variation_ = player_robot_distance - current_robot_player_distance_;
    current_robot_player_distance_ = player_robot_distance;
}

bool PlayerActivityMonitoring::PlayerActivityMonitor::isStationary(geometry_msgs::Point new_pos){
    // If the player has not moved of a 
    double displacement = sqrt(pow(new_pos.x - current_player_position_.x, 2) + pow(new_pos.y - current_player_position_.y, 2));
    // ROS_INFO("Displacement: %.2f", displacement);
    if(displacement < STATIONARY_TRESHOLD){
        return true;
    }
    return false;
}

int PlayerActivityMonitoring::PlayerActivityMonitor::getClosestTowerIndex(){
    // Returns the index of the closest tower to the player
    int closest_index = 0;
    int i;
    for(i=1; i<NUM_TOWERS; i++){
        if(current_player_tower_distance_[i] < current_player_tower_distance_[closest_index]){
            closest_index = i;
        }
    }
    return closest_index;
}

int PlayerActivityMonitoring::PlayerActivityMonitor::evaluatePlayerTargetTower(geometry_msgs::Point new_pos){
    // Estimate which tower is the player going

    // Direction of player motion expressed as angle
    double motion_direction = atan2(current_player_position_.y - new_pos.y, current_player_position_.x - new_pos.x) + M_PI;

    int i;
    int best_guess;
    double best_diff = 10.0; // bounded to be in [0, 2pi]
    for(i=0; i<NUM_TOWERS; i++){
        // Direction of tower i from the old position of the player, expressed in angle
        double tower_direction = atan2(current_player_position_.y - tower_pos_[i].y, current_player_position_.x - tower_pos_[i].x) + M_PI;
        double diff = fabs(tower_direction - motion_direction);
        if(diff < best_diff){
            // If the motion direction matches best the direction of this tower, update guess
            best_diff = diff;
            best_guess = i;
        }
    }
    return best_guess;
}

geometry_msgs::Point PlayerActivityMonitoring::PlayerActivityMonitor::transformPointToMap(geometry_msgs::PointStamped in){
    // geometry_msgs::PointStamped tmp;

    // listener_.waitForTransform("/base_link", "/map", ros::Time(0), ros::Duration(10.0));
    // try{
    //     listener_.transformPoint("/map", ros::Time(0), in, "/map",  tmp);
    // }
    // catch(tf::TransformException& ex){
    //     ROS_ERROR("Received an exception trying to transform a point : %s", ex.what());
    // }
    // return tmp.point;

    geometry_msgs::Point out;

    updateCurrentRobotPos();

    double angle_wrt_robot = atan2(in.point.y, in.point.x); // in is wrt robot frame
    double distance = sqrt(pow(in.point.x, 2.0) + pow(in.point.y, 2.0));
    out.x = cos(angle_wrt_robot + current_robot_rotation_wrt_map_) * distance + current_robot_pos_.x;
    out.y = sin(angle_wrt_robot + current_robot_rotation_wrt_map_) * distance + current_robot_pos_.y;

    return out;
}

void PlayerActivityMonitoring::PlayerActivityMonitor::publishActivityMsg(){
    activity_msg_.player_tower_distances = current_player_tower_distance_;
    activity_msg_.robot_tower_distances  = current_robot_tower_distance_;
    activity_msg_.player_robot_distance = current_robot_player_distance_;

    activity_msg_.player_tower_distances_variation = current_player_tower_distance_variation_;
    activity_msg_.robot_tower_distances_variation = current_robot_tower_distance_variation_;
    activity_msg_.player_robot_distance_variation = current_robot_player_distance_variation_;

    activity_msg_.player_position_wrt_map = current_player_position_;
    activity_msg_.robot_position_wrt_map = current_robot_pos_;
    activity_msg_.robot_rotation_wrt_map = current_robot_rotation_wrt_map_;

    activity_msg_.velocity_projections = robot_velocity_projections_;

    activity_pub_.publish(activity_msg_);
}

void PlayerActivityMonitoring::PlayerActivityMonitor::updateVelocityProjections(){
    int i;
    double motion_angle = atan2(current_robot_vel_.y, current_robot_vel_.x) + M_PI;
    for(i=0; i<NUM_TOWERS; i++){
        double tower_angle = atan2(current_robot_pos_.y - tower_pos_[i].y, current_robot_pos_.x - tower_pos_[i].x) + M_PI;
        double angle_diff = tower_angle - motion_angle;

        // ROS_INFO("TOWER_ANGLE: %.2f MOTIONANGLE: %.2f DIFFANGLE: %.2f", tower_angle, motion_angle, angle_diff);

        robot_velocity_projections_[i] = -(current_robot_speed_ * cos(angle_diff));
    }
}

void PlayerActivityMonitoring::PlayerActivityMonitor::velCallback(const geometry_msgs::Twist vel){
    // Callback for robot velocity
    double speed = sqrt(pow(vel.linear.x, 2.0) + pow(vel.linear.y, 2.0));
    // ROS_WARN("CURRENT SPEED: %.2f", speed);
    if(speed > VEL_TRESHOLD){
        current_robot_speed_ = speed;
        // current_robot_speed_ = 1;
    }
    else{
        current_robot_speed_ = 0.0;
    }
    // Express the vector in the map frame
    current_robot_vel_ = rotateToMap(vel.linear);
}

geometry_msgs::Vector3 PlayerActivityMonitoring::PlayerActivityMonitor::rotateToMap(geometry_msgs::Vector3 vector){
    // Utility function to express velocity in the map frame
    geometry_msgs::Vector3 output;

    double current_orientation = atan2(vector.y, vector.x);
    double magnitude = current_robot_speed_;
    double new_orientation = current_orientation + current_robot_rotation_wrt_map_;

    output.x = magnitude * cos(new_orientation);
    output.y = magnitude * sin(new_orientation);
    output.z = 0.0;
    return output;
}


void PlayerActivityMonitoring::PlayerActivityMonitor::updateCurrentRobotPos(){
       
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

void PlayerActivityMonitoring::PlayerActivityMonitor::towerPosCallback(const planning::TowerPositions message){
    for(int i=0; i<NUM_TOWERS; i++){
        tower_pos_[i] = message.towers[i];
    }
}

int PlayerActivityMonitoring::PlayerActivityMonitor::getMostRiskyTower(){
    int risk_index = 0;
    int i;

    for(i=1; i<NUM_TOWERS; i++){
        if(robot_velocity_projections_[i] > robot_velocity_projections_[risk_index]){
            risk_index = i;
        }
    }

    return risk_index;
}

void PlayerActivityMonitoring::PlayerActivityMonitor::publishPlayerTargetMarker(){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    marker.type = visualization_msgs::Marker::SPHERE;
    
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.0;
    
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;

    int tower_index = getMostRiskyTower();
    marker.pose.position.x = tower_pos_[tower_index].x;
    marker.pose.position.y = tower_pos_[tower_index].y;
    marker.pose.position.z = tower_pos_[tower_index].z;
    
    player_target_pub_.publish(marker);
}

void PlayerActivityMonitoring::PlayerActivityMonitor::publishRiskyTower(){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    marker.type = visualization_msgs::Marker::SPHERE;
    
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.0;
    
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    marker.pose.position.x = tower_pos_[activity_msg_.target_tower].x;
    marker.pose.position.y = tower_pos_[activity_msg_.target_tower].y;
    marker.pose.position.z = tower_pos_[activity_msg_.target_tower].z;
    
    player_target_pub_.publish(marker);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "player_activity_monitor_node");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    PlayerActivityMonitoring::PlayerActivityMonitor monitor;

    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }
}