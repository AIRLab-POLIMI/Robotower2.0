#include <ros/ros.h>
#include "planning/locomotion_planning.h"
#include "planning/TowerPositions.h"

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Twist.h>


LocomotionPlanning::LocomotionPlanner::LocomotionPlanner(){
    
    if (!nh_.getParam("/planning/vel_topic_pub", vel_topic_)){
        ROS_ERROR("LOCOMOTION PLANNER: could not read 'vel_topic' from rosparam!");
        exit(-1);
    }

    if (!nh_.getParam("/planning/behavior_topic", steering_topic_)){
        ROS_ERROR("LOCOMOTION PLANNER: could not read 'steering_topic' from rosparam!");
        exit(-1);
    }

            
    if (!nh_.getParam("/planning/tower_rectangle_topic", tower_rectangle_topic_)){
        ROS_ERROR("STEERING BEHAVIOR MANAGER: could not read 'tower_rectangle_topic' from rosparam!");
        exit(-1);
    }


    // Get initial position configuration for towers
    towers_.resize(4);
    std::vector<float> tower_pos;
    for(int i=0; i<4; i++){
        std::string param_name;
        param_name = "/tower_" + std::to_string(i+1);
        if (!nh_.getParam(param_name, tower_pos)){
            ROS_ERROR("STEERING BEHAVIOR MANAGER: could not read 'tower' from rosparam!");
            exit(-1);
        }
        geometry_msgs::Point32 pos;
        pos.x = tower_pos[0];
        pos.y = tower_pos[1];
        towers_[i] = pos;
    }

    tower_rectangle_sub_ = nh_.subscribe(tower_rectangle_topic_, 1, &LocomotionPlanning::LocomotionPlanner::towerRectangleCallback, this);
    steering_sub_ = nh_.subscribe(steering_topic_, 5, &LocomotionPlanning::LocomotionPlanner::steeringCallback, this);
    // player_model_sub_ = nh_.subscribe("/player_model", 1, &LocomotionPlanning::LocomotionPlanner::playerModelCallback, this);
    reset_sub_ = nh_.subscribe("/game_manager/reset", 1, &LocomotionPlanning::LocomotionPlanner::resetCallback, this);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>(vel_topic_, 1);
    tower_pos_pub_ = nh_.advertise<planning::TowerPositions>("/tower_pos", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/target_steering", 1);

    current_steering_.behavior_code = -1; // Initialized to invalid code
    planning_ = false;
    player_model_ = 0.0;
}

void LocomotionPlanning::LocomotionPlanner::towerRectangleCallback(const geometry_msgs::PolygonStamped& poly){
    std::vector<geometry_msgs::Point32> points;
    points.resize(4);
    for(int i=0; i<4; i++){
        points[i] = poly.polygon.points[i];
    }
    updateTowerPositions(points);
    publishTowerPositions();
}

void LocomotionPlanning::LocomotionPlanner::updateTowerPositions(std::vector<geometry_msgs::Point32> points){
    for(int i=0; i<points.size(); i++){
        geometry_msgs::Point32 point = points[i];
        int index = matchTowerIndex(point);
        towers_[index].x = point.x;
        towers_[index].y = point.y;
    }
}

int LocomotionPlanning::LocomotionPlanner::matchTowerIndex(geometry_msgs::Point32 point){
    // Match the closest tower with the point given
    float min_dist = 100.0;
    int min_index;

    for(int i=0; i<towers_.size(); i++){
        float distance = pow(point.x - towers_[i].x, 2) + pow(point.y - towers_[i].y, 2);
        if(distance < min_dist){
            min_index = i;
            min_dist = distance;
        }
    }
    return min_index;
}

void LocomotionPlanning::LocomotionPlanner::publishTowerPositions(){
    planning::TowerPositions positions;
    positions.towers = towers_;
    tower_pos_pub_.publish(positions);
}

void LocomotionPlanning::LocomotionPlanner::steeringCallback(const planning::SteeringBehaviorEncoded& steering){
    if(steering.behavior_code != current_steering_.behavior_code){
        current_steering_ = steering;
		current_behavior_ = steering_factory_.generateSteeringBehavior(current_steering_);        
		planning_ = true;
        
        current_behavior_ -> setSlowingRadius(vehicle_.getSlowingRadius());
        current_behavior_ -> setMaxSpeed(vehicle_.getMaxSpeed());
        current_behavior_ -> setDeceptionTargetDistance(vehicle_.getDeceptionChangeTargetDistance());

        vehicle_.setSteeringBehavior(current_behavior_);
        // vehicle_.resetParams();
    }
}


void LocomotionPlanning::LocomotionPlanner::updateLoop(){
    if(planning_){

        current_behavior_->updateTargetPos(towers_);
        publishTarget(current_behavior_->getTarget());

        vehicle_.updateCurrentPos();
        
        geometry_msgs::Twist cmd = vehicle_.generateCommandVel();
        
        // Rotate to consider current rotation wrt map
        geometry_msgs::Twist rotated_cmd = vehicle_.alignCommand(cmd);
        vel_pub_.publish(rotated_cmd);
    }
    else{
        ROS_INFO("Waiting for steering behavior");
    }
}

void LocomotionPlanning::LocomotionPlanner::playerModelCallback(activity_monitor::PlayerModel model){
    player_model_ = model.cumulative_hyperparam;
    vehicle_.updateKinematicProperties(model);
}

void LocomotionPlanning::LocomotionPlanner::resetCallback(std_msgs::Bool reset){
    vehicle_.resetParams();
}

void LocomotionPlanning::LocomotionPlanner::publishTarget(geometry_msgs::Point32 target){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    marker.type = visualization_msgs::Marker::SPHERE;
    
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.0;
    
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    marker.pose.position.x = target.x;
    marker.pose.position.y = target.y;
    marker.pose.position.z = target.z;

    marker_pub_.publish(marker);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "planning_locomotion_node");
    ros::NodeHandle nh;
    ros::Rate rate(30);

    LocomotionPlanning::LocomotionPlanner planner;

    while(ros::ok()){
        planner.updateLoop();
        ros::spinOnce();
        rate.sleep();
    }
}
