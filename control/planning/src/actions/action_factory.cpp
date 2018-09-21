#include "action/action_lib.h"
#include "action/capture_tower.h"
#include "action/escape.h"
#include "planning/ActionEncoded.h"

#include <ros/ros.h>
#include <cmath>
#include <string>

#define CAPTURE_TOWER 1
#define ESCAPE 2

Action::ActionFactory::ActionFactory(){
    if (!nh_.getParam("/planning/tower_pos_topic", tower_topic_)){
        ROS_ERROR("ACTION FACTORY: could not read 'tower_topic' from rosparam!");
        exit(-1);
    }


    if (!nh_.getParam("/planning/tower_rectangle_topic", tower_rectangle_topic_)){
        ROS_ERROR("STEERING BEHAVIOR MANAGER: could not read 'tower_rectangle_topic' from rosparam!");
        exit(-1);
    }

    scan_sub_ = nh_.subscribe("/scan", 1, &Action::ActionFactory::laserCallback, this);
    tower_rectangle_sub_ = nh_.subscribe(tower_rectangle_topic_, 1, &Action::ActionFactory::towerRectangleCallback, this);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/target_steering", 1);

    ready_ = false;
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
        ready_ = true;

    }

    std::vector<float> ran(1000, 5.0);
    current_scan_.ranges = ran;
    current_scan_.ranges[150] = 0.6;
    laserCallback(current_scan_);    
}

void Action::ActionFactory::towerRectangleCallback(const geometry_msgs::PolygonStamped& poly){
    std::vector<geometry_msgs::Point32> points;
    points.resize(4);
    for(int i=0; i<4; i++){
        points[i] = poly.polygon.points[i];
    }
    updateTowerPositions(points);
}

void Action::ActionFactory::updateTowerPositions(std::vector<geometry_msgs::Point32> points){
    for(int i=0; i<points.size(); i++){
        geometry_msgs::Point32 point = points[i];
        int index = matchTowerIndex(point);
        towers_[index].x = point.x;
        towers_[index].y = point.y;
    }
       
}

int Action::ActionFactory::matchTowerIndex(geometry_msgs::Point32 point){
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

void Action::ActionFactory::laserCallback(const sensor_msgs::LaserScan scan){
    current_scan_ = scan;
}


Action::AbstractAction* Action::ActionFactory::generateAction(planning::ActionEncoded action_msg, int tower_index){
    if(ready_){
        switch(action_msg.action_code){
            case CAPTURE_TOWER:
                return generateCaptureAction(tower_index);
            case ESCAPE:
                return generateEscapeAction(action_msg);
        }
    }
    else{
        throw "Waiting for towers!!!";
    }
}

Action::AbstractAction* Action::ActionFactory::generateCaptureAction(int tower_index){
    updateCurrentPos();
    geometry_msgs::Point32 target;
    float min_dist = 100.0;
    int min_index;
    for(int i=0; i<4; i++){
        float dist = pow(current_pos_.x - towers_[i].x, 2) + pow(current_pos_.y - towers_[i].y, 2);
        if(dist < min_dist){
            min_dist = dist;
            min_index = i;
        }
    }
    target = towers_[min_index];   
    // TODO remove
    target = towers_[3];

    ROS_ERROR("Tower :%d", min_index);
    publishTarget(target);
    last_tower_index_ = min_index;
    
    return new Action::CaptureTower(tower_index, target);
}

void Action::ActionFactory::publishTarget(geometry_msgs::Point32 target){
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

    marker.pose.position.x = target.x;
    marker.pose.position.y = target.y;
    marker.pose.position.z = target.z;

    marker_pub_.publish(marker);
}

Action::AbstractAction* Action::ActionFactory::generateEscapeAction(planning::ActionEncoded action_msg){
    geometry_msgs::Point32 target;

    float obstacle_distance = current_scan_.ranges[action_msg.danger_index];
    float obstacle_angle = (action_msg.danger_index * (2*M_PI/1000.0));

    target.x = current_pos_.x + cos(obstacle_angle)*obstacle_distance;
    target.y = current_pos_.y + sin(obstacle_angle)*obstacle_distance;
    target.z = 0.0;
    
    // target = towers_[last_tower_index_];
    return new Action::Escape(target);
}

void Action::ActionFactory::updateCurrentPos(const geometry_msgs::Pose& msg){
    // TODO get robot pose using ROS
    current_pos_.x = msg.position.x;
    current_pos_.y = msg.position.y;
    current_pos_.z = msg.position.z;
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
            
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
}