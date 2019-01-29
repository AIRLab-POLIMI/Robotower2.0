#include <ros/ros.h>
#include "planning2/locomotion_planner.h"
#include "planning2/behaviors/steering_behavior.h"
#include "planning2/behaviors/flee.h"
#include "planning2/behaviors/arrival.h"
#include "planning2/behaviors/stop.h"
#include "planning2/behaviors/seek.h"

#include <geometry_msgs/Twist.h>

LocomotionPlanning::LocomotionPlanner::LocomotionPlanner(){
    player_ = 0.0;

    initMap();
    initTowers();

    actionSub_ = nh_.subscribe("action", 1, &LocomotionPlanning::LocomotionPlanner::actionCallback, this);
    towerRectangleSub_ = nh_.subscribe("/tower_rectangle", 1, &LocomotionPlanning::LocomotionPlanner::towerRectangleCallback, this);
    gameStateSub_ = nh_.subscribe("/player_activity", 1, &LocomotionPlanning::LocomotionPlanner::gameStateCallback, this);
    
    cmdVelPub_ = nh_.advertise<geometry_msgs::Twist>("/game_navigation/unsafe/cmd_vel", 1);
    towerPosPpub_ = nh_.advertise<planning2::TowerArray>("/tower_pos", 1);
    markerPub_ = nh_.advertise<visualization_msgs::Marker>("/target", 1);
    currentBehavior_ = nullptr;
    currentAction_.action_code = -1; // Ids start from zero, this is to set up to a non possible id
}

void LocomotionPlanning::LocomotionPlanner::initMap(){
    LocomotionPlanning::SteeringBehavior* seek = new LocomotionPlanning::Seek();
    LocomotionPlanning::SteeringBehavior* arrival = new LocomotionPlanning::Arrival();
    LocomotionPlanning::SteeringBehavior* stop = new LocomotionPlanning::Stop();
    LocomotionPlanning::SteeringBehavior* flee = new LocomotionPlanning::Flee();
    
    std::vector<SteeringBehavior*> captureTowerBehaviors;
    captureTowerBehaviors.push_back(seek);
    captureTowerBehaviors.push_back(arrival);

    std::vector<SteeringBehavior*> escapeBehaviors;
    escapeBehaviors.push_back(flee);

    std::vector<SteeringBehavior*> waitBehaviors;
    waitBehaviors.push_back(stop);
   
    behaviors_.insert({0, captureTowerBehaviors});
    behaviors_.insert({2, escapeBehaviors});
    behaviors_.insert({3, waitBehaviors});
}

void LocomotionPlanning::LocomotionPlanner::initTowers(){
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
        ROS_ERROR_STREAM(towers_[i]);
    }

}

void LocomotionPlanning::LocomotionPlanner::gameStateCallback(activity_monitor::Activity activityMessage){
    currentGameState_ = activityMessage;
    currentPos_.x = currentGameState_.robot_position_wrt_map.x;
    currentPos_.y = currentGameState_.robot_position_wrt_map.y;
    currentRotation_ = currentGameState_.robot_rotation_wrt_map;
}

LocomotionPlanning::SteeringBehavior* LocomotionPlanning::LocomotionPlanner::getBestBehavior(std::vector<SteeringBehavior*> behaviors){
    int i;
    int bestScore = 0.0;
    int bestBehaviorIndex = 0;

    for(i=0; i<behaviors.size(); i++){
        if(behaviors[i]->evaluate(player_) > bestScore){
            bestBehaviorIndex = i;
            bestScore = behaviors[i]->evaluate(player_);
        }
    }
    return behaviors[bestBehaviorIndex];
}

void LocomotionPlanning::LocomotionPlanner::actionCallback(planning2::Action aMsg){
    if(aMsg.action_code == currentAction_.action_code){
        return;
    }
    currentAction_ = aMsg;
    targetQueue_.resize(aMsg.targets.size());
    towerIndexes_.resize(aMsg.tower_indexes.size());

    targetQueue_ = aMsg.targets;

    currentIndex_ = 0;
    currentTargetIndex_ = 0;
    popTarget();
    currentBehavior_ = getBestBehavior(behaviors_.at(aMsg.action_code));
    
    currentBehavior_ -> setTarget(currentTarget_.target);

    vehicle_.setBehavior(currentBehavior_);
}

bool LocomotionPlanning::LocomotionPlanner::popTarget(){
    if(currentIndex_ < targetQueue_.size()){
        currentTarget_ = targetQueue_[currentIndex_];
        currentIndex_ += 1;
        if(currentTarget_.is_tower){
            ROS_ERROR("UPDATING TARGET WITH TOWER POSITION %d", currentTarget_.tower_index);
            currentTarget_.target = towers_[currentTarget_.tower_index];
            ROS_ERROR_STREAM(currentTarget_.target);
            ROS_ERROR_STREAM(towers_[currentTarget_.tower_index]);
        }
        return true;
    }
    return false;
}

void LocomotionPlanning::LocomotionPlanner::towerRectangleCallback(const geometry_msgs::PolygonStamped& poly){
    std::vector<geometry_msgs::Point32> points;
    points.resize(4);
    for(int i=0; i<4; i++){
        points[i] = poly.polygon.points[i];
    }
    updateTowerPositions(points);
    publishTowerPositions();
    if(currentTarget_.is_tower){
        // Update with new position of the tower
        currentBehavior_->setTarget(towers_[currentTarget_.tower_index]);
    }
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
    planning2::TowerArray positions;
    positions.towers = towers_;
    towerPosPpub_.publish(positions);
}


void LocomotionPlanning::LocomotionPlanner::loop(){
    if(currentBehavior_ == nullptr){
        ROS_WARN("Waiting for behavior..");
        return;
    }
    else{
        ROS_INFO_STREAM(currentBehavior_->getName());
        
        vehicle_.locateVehicle(currentPos_, currentRotation_);
        cmdVelPub_.publish(vehicle_.generateCommandVel());
        publishTarget(currentBehavior_->getTarget());
    }
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

    markerPub_.publish(marker);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "locomotion_planner_node");
    ros::NodeHandle nh;
    ros::Rate rate(30);

    LocomotionPlanning::LocomotionPlanner lPlanner;

    while(ros::ok()){
        lPlanner.loop();
        ros::spinOnce();
        rate.sleep();
    }
}