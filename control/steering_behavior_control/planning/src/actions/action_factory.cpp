#include "action/action_lib.h"
#include "action/capture_tower.h"
#include "action/escape.h"
#include "action/deceive.h"
#include "planning/ActionEncoded.h"
#include <steering_behavior/steering_behavior.h>

#include <ros/ros.h>
#include <cmath>
#include <string>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3.h>

#define CAPTURE_TOWER 1
#define ESCAPE 2
#define DECEIVE 3

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
        updateCurrentPos();
        switch(action_msg.action_code){
            case CAPTURE_TOWER:
                return generateCaptureAction(tower_index);
            case ESCAPE:
                return generateEscapeAction(action_msg);
            case DECEIVE:
                return generateDeceivingAction(action_msg);
        }
    }
    else{
        throw "Waiting for towers!!!";
    }
}

Action::AbstractAction* Action::ActionFactory::generateCaptureAction(int tower_index){
    geometry_msgs::Point32 target;
    float min_dist = 100.0;
    int min_index;
    for(int i=0; i<4; i++){
        float dist = pow(current_pos_.x - towers_[i].x, 2) + pow(current_pos_.y - towers_[i].y, 2);
        if(dist < min_dist && i!=last_tower_index_){
            min_dist = dist;
            min_index = i;
        }
    }
    target = towers_[min_index];   

    ROS_ERROR("Tower :%d", min_index);
    // publishTarget(target);
    last_tower_index_ = min_index;
    
    return new Action::CaptureTower(tower_index, target);
}

Action::AbstractAction* Action::ActionFactory::generateEscapeAction(planning::ActionEncoded action_msg){
    geometry_msgs::Point32 target;

    float obstacle_distance = current_scan_.ranges[action_msg.danger_index];
    float obstacle_angle = (action_msg.danger_index * (2*M_PI/1000.0));

    target.x = current_pos_.x + cos(obstacle_angle + current_rotation_wrt_map_)*obstacle_distance;
    target.y = current_pos_.y + sin(obstacle_angle + current_rotation_wrt_map_)*obstacle_distance;
    target.z = 0.0;
    
    return new Action::Escape(target);
}

Action::AbstractAction* Action::ActionFactory::generateDeceivingAction(planning::ActionEncoded action_msg){
    // TODO get tower from Laura's message
    geometry_msgs::Point32 real_target = towers_[2];
    geometry_msgs::Point32 deceptive_tower = towers_[3];
    
    std::vector<geometry_msgs::Point32> targets = generateDeceptiveTargets(real_target, deceptive_tower); // DOES THE SERPENTONE THING
    // std::vector<geometry_msgs::Point32> targets = generateDeceptiveTargetsBIS(real_target, deceptive_tower); // GO TO MIDDLE POINT AND DECIDES

    return new Action::Deceive(targets);
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
    // publishTarget(targets[0]);
    return targets;
}

std::vector<geometry_msgs::Point32> Action::ActionFactory::generateDeceptiveTargets(geometry_msgs::Point32 real_target, geometry_msgs::Point32 fake_target){
    // DOES THE SERPENTONE THING
    geometry_msgs::Vector3 vector_fake;
    float magnitude_fake;
    vector_fake = SteeringBehavior::VectorUtility::vector_difference(current_pos_, fake_target);
    magnitude_fake = SteeringBehavior::VectorUtility::magnitude(vector_fake);

    geometry_msgs::Vector3 vector_true;
    float magnitude_true;
    vector_true = SteeringBehavior::VectorUtility::vector_difference(current_pos_, real_target);
    magnitude_true = SteeringBehavior::VectorUtility::magnitude(vector_true);

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
    targets.resize(3);
    targets[0] = p1;
    targets[1] = p2;
    targets[2] = p3;

    return targets;
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
