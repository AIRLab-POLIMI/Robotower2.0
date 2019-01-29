#include "action/action_lib.h"
#include "action/capture_tower.h"
#include "action/escape.h"
#include "action/deceive.h"
#include "planning/ActionEncoded.h"
#include <steering_behavior/steering_behavior.h>
#include <behavior_control/Goal.h>
#include <behavior_control/GoalService.h>

#include <ros/ros.h>
#include <cmath>
#include <string>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int8.h>

#define CAPTURE_TOWER 1
#define ESCAPE 2
#define DECEIVE 3

Action::ActionFactory::ActionFactory(){
    scan_sub_ = nh_.subscribe("/scan", 1, &Action::ActionFactory::laserCallback, this);
    // goal_sub_ = nh_.subscribe("/game/goal", 1, &Action::ActionFactory::goalCallback, this); 
    goal_sub_ = nh_.subscribe("/game/goal", 1, &Action::ActionFactory::goalCallback, this); 
    parameter_sub_ = nh_.subscribe("/parameter_id", 1, &Action::ActionFactory::parameterCallback, this); 
    start_interaction_pub_ = nh_.advertise<std_msgs::Int8>("/start_interaction", 1);
    abort_interaction_pub_ = nh_.advertise<std_msgs::Int8>("/abort_interaction", 1);

    start_attack_pub_ = nh_.advertise<std_msgs::Int8>("/start_attack", 1);
    end_attack_pub_ = nh_.advertise<std_msgs::Int8>("/end_attack", 1);

    goal_service_client_ = nh_.serviceClient<behavior_control::GoalService>("goal_service");
    ready_ = false;

    // Initializing tower position
    towers_.resize(4);
    std::vector<float> tower_pos;
    for(int i=0; i<4; i++){
        std::string param_name;
        param_name = "/tower_" + std::to_string(i+1);
        if (!nh_.getParam(param_name, tower_pos)){
            ROS_ERROR("ACTION FACTORY: could not read 'tower' from rosparam!");
            exit(-1);
        }
        geometry_msgs::Point32 pos;
        pos.x = tower_pos[0];
        pos.y = tower_pos[1];
        towers_[i] = pos;
    }

    current_goal_ = -1; // Initializing to invalid index

}

// void Action::ActionFactory::goalCallback(const behavior_with_deception::Goal& goal_msg){
//     // Goals are published in range [1,4] but the array has domain [0,3] => subtract 1
//     ready_ = true;
//     current_goal_ = goal_msg.tower_number - 1;
// }

void Action::ActionFactory::goalCallback(const behavior_control::Goal& goal_msg){
    // Goals are published in range [1,4] but the array has domain [0,3] => subtract 1
    ready_ = true;
    current_goal_ = goal_msg.tower_number - 1;
}

void Action::ActionFactory::laserCallback(const sensor_msgs::LaserScan scan){
    current_scan_ = scan;
}

void Action::ActionFactory::parameterCallback(const std_msgs::Int8 msg){
    parameter_id_ = msg.data;
}


Action::AbstractAction* Action::ActionFactory::generateAction(planning::ActionEncoded action_msg){
    ready_ = true;
    if(ready_){
        switch(action_msg.action_code){
            case CAPTURE_TOWER:
                return generateCaptureAction();
            case ESCAPE:
                return generateEscapeAction(action_msg);
            case DECEIVE:
                return generateDeceivingAction(action_msg);
        }
    }
    else{
        throw "Waiting for goal!!!";
    }
}

Action::AbstractAction* Action::ActionFactory::generateCaptureAction(){
    geometry_msgs::Point32 target;

    ROS_WARN("Waiting for service");
    goal_service_client_.waitForExistence();
    ROS_WARN("Service Ready!");
    behavior_control::GoalService goal_srv;

    goal_srv.request.parameter_id = parameter_id_;
    if(goal_service_client_.call(goal_srv)){
        current_goal_ = goal_srv.response.tower_id - 1;
    }
    else{
        ROS_ERROR("Failed to call goal service");
    }
    
    target = towers_[current_goal_];

    ROS_ERROR("Tower :%d", current_goal_ + 1);

    std_msgs::Int8 interaction_msg;
    interaction_msg.data = current_goal_;
    start_interaction_pub_.publish(interaction_msg);
    start_attack_pub_.publish(interaction_msg);
    
    return new Action::CaptureTower(current_goal_, target);
}

Action::AbstractAction* Action::ActionFactory::generateEscapeAction(planning::ActionEncoded action_msg){
    // Update out current position to interpret laser scan correctly
    updateCurrentPos();
    geometry_msgs::Point32 target;

    float obstacle_distance = current_scan_.ranges[action_msg.danger_index];
    float obstacle_angle = (action_msg.danger_index * (2*M_PI/1000.0));

    target.x = current_pos_.x + cos(obstacle_angle + current_rotation_wrt_map_)*obstacle_distance;
    target.y = current_pos_.y + sin(obstacle_angle + current_rotation_wrt_map_)*obstacle_distance;
    target.z = 0.0;

    std_msgs::Int8 interaction_msg;
    abort_interaction_pub_.publish(interaction_msg);
    end_attack_pub_.publish(0);
    
    return new Action::Escape(target);
}

Action::AbstractAction* Action::ActionFactory::generateDeceivingAction(planning::ActionEncoded action_msg){
    // TODO get tower from Laura's message
    geometry_msgs::Point32 real_target = towers_[action_msg.real_target];
    geometry_msgs::Point32 deceptive_tower = towers_[action_msg.fake_target];
    std::vector<geometry_msgs::Point32> targets;
	if(action_msg.type == 3){
        ROS_WARN("PREPARING FOR TYPE 3 DECEPTION");
    	targets = generateDeceptiveTargets(real_target, deceptive_tower); // DOES THE SERPENTONE THING
	}
	else{
        ROS_WARN("PREPARING FOR TYPE 1/2 DECEPTION");
        targets = generateDeceptiveTargetsBIS(real_target, deceptive_tower); // GO TO MIDDLE POINT AND DECIDES
    }

    std_msgs::Int8 interaction_msg;
    abort_interaction_pub_.publish(interaction_msg);

    return new Action::Deceive(targets, action_msg.real_target);
}

std::vector<geometry_msgs::Point32> Action::ActionFactory::generateDeceptiveTargetsBIS(geometry_msgs::Point32 real_target, geometry_msgs::Point32 deceptive_tower){
    // GOES TO THE MIDDLE POINT BETWEEN TWO TOWERS, THEN TURNS TO ONE
    geometry_msgs::Point32 fake_target;

    fake_target.x = (real_target.x + deceptive_tower.x)/2.0;
    fake_target.y = (real_target.y + deceptive_tower.y)/2.0;

    geometry_msgs::Vector3 vector;
    float magnitude;
    vector = SteeringBehavior::VectorUtility::vector_difference(current_pos_, fake_target);
    magnitude = SteeringBehavior::VectorUtility::magnitude(vector);


    vector = SteeringBehavior::VectorUtility::scalar_multiply(SteeringBehavior::VectorUtility::normalize(vector), magnitude*0.8);
    fake_target.x = current_pos_.x + vector.x;
    fake_target.y = current_pos_.y + vector.y;

    ROS_ERROR("FAKE TARGET x:%.2f y:%.2f", fake_target.x, fake_target.y);
    ROS_ERROR("REAL TARGET x:%.2f y:%.2f", real_target.x, real_target.y);
    
    std::vector<geometry_msgs::Point32> targets;
    targets.resize(2);
    targets[0] = fake_target;
    targets[1] = real_target;
    return targets;
}

std::vector<geometry_msgs::Point32> Action::ActionFactory::generateDeceptiveTargets(geometry_msgs::Point32 real_target, geometry_msgs::Point32 fake_target){
    // DOES THE SERPENTONE THING

    // Vector representing shortest path to the fake tower
    geometry_msgs::Vector3 vector_fake;
    float magnitude_fake;
    vector_fake = SteeringBehavior::VectorUtility::vector_difference(current_pos_, fake_target);
    magnitude_fake = SteeringBehavior::VectorUtility::magnitude(vector_fake);

    // Vector representing shortest path to the real tower
    geometry_msgs::Vector3 vector_true;
    float magnitude_true;
    vector_true = SteeringBehavior::VectorUtility::vector_difference(current_pos_, real_target);
    magnitude_true = SteeringBehavior::VectorUtility::magnitude(vector_true);

    // Calculate two points: one at 0.4 of the shortest path to the real target, one at 0.6 of the shortest path to the fake target
    geometry_msgs::Vector3 vect1 = SteeringBehavior::VectorUtility::scalar_multiply(SteeringBehavior::VectorUtility::normalize(vector_true), magnitude_true*0.4);
    geometry_msgs::Vector3 vect2 = SteeringBehavior::VectorUtility::scalar_multiply(SteeringBehavior::VectorUtility::normalize(vector_fake), magnitude_fake*0.6);

    geometry_msgs::Point32 p1;
    p1.x = current_pos_.x + vect1.x; 
    p1.y = current_pos_.y + vect1.y; 

    geometry_msgs::Point32 p2;
    p2.x = current_pos_.x + vect2.x; 
    p2.y = current_pos_.y + vect2.y;
    // p2 = fake_target; 

    geometry_msgs::Point32 p3;
    p3 = real_target;

    std::vector<geometry_msgs::Point32> targets;
    // targets.resize(3);
    // targets[0] = p1;
    // targets[1] = p2;
    // targets[2] = p3;
    targets.resize(2);
    targets[0] = p2;
    targets[1] = p3;
    // targets[2] = p3;

    return targets;
}

void Action::ActionFactory::updateCurrentPos(){
        /**
        Gets robot global position. That is, performs a TF transformation from /base_link to /map and returns
        x,y and theta.
        OUTPUTS:
        @ a 3D-numpy array defined as: [x, y, theta] w.r.t /map.
        **/
        tf::StampedTransform transform;
        try{
            listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
            current_pos_.x = transform.getOrigin().getX();
            current_pos_.y = transform.getOrigin().getY();
            current_pos_.z = transform.getOrigin().getZ();

            double roll, pitch, yaw;
            tf::Quaternion quat = transform.getRotation();            
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            
            current_rotation_wrt_map_ = yaw;
            
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
}
