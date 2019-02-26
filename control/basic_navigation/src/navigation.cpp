#include <ros/ros.h>
#include "basic_navigation/navigation.h"
#include <behavior_control/Goal.h>
#include <behavior_control/GoalService.h>
#include <visualization_msgs/Marker.h>

#include <string>

Navigation::BasicNavigation::BasicNavigation(){
    // Constructor
    if (!nh_.getParam("/num_towers", num_towers_)){
        ROS_ERROR("BEHAVIOR MANAGER: could not read 'num_towers' from rosparam!");
        exit(-1);
    }

    if (!nh_.getParam("/navigation/tolerance_delta", toleranceDelta_)){
        ROS_ERROR("NAVIGATION: could not read 'tolerance_delta' from rosparam!");
        toleranceDelta_ = 20;
        // exit(-1);
    }

    if (!nh_.getParam("/navigation/tolerance_speed", toleranceSpeed_)){
        ROS_ERROR("NAVIGATION: could not read 'tolerance_speed' from rosparam!");
        toleranceSpeed_ = 0.05;
        // exit(-1);
    }

    if (!nh_.getParam("/navigation/max_speed", maxSpeed_)){
        ROS_ERROR("NAVIGATION: could not read 'max_speed' from rosparam!");
        maxSpeed_ = 0.7;
        // exit(-1);
    }

    if (!nh_.getParam("/navigation/acceleration", acceleration_)){
        ROS_ERROR("NAVIGATION: could not read 'acceleration' from rosparam!");
        acceleration_ = 0.05;
        // exit(-1);
    }

    if (!nh_.getParam("/navigation/vel_topic", velTopic_)){
        ROS_ERROR("NAVIGATION: could not read 'vel_topic' from rosparam!");
        exit(-1);
    }

    if (!nh_.getParam("/navigation/player_topic", playerTopic_)){
        ROS_ERROR("NAVIGATION: could not read 'player_topic' from rosparam!");
        exit(-1);
    }

    if (!nh_.getParam("/navigation/tower_rectangle_topic", towerRectangleTopic_)){
        ROS_ERROR("NAVIGATION: could not read 'tower_rectangle_topic' from rosparam!");
        exit(-1);
    }

    if (!nh_.getParam("/navigation/vel_pub_topic", velPubTopic_)){
        ROS_ERROR("NAVIGATION: could not read 'vel_pub_topic' from rosparam!");
        exit(-1);
    }

    if (!nh_.getParam("/navigation/collision_distance_treshold", collisionDistanceTreshold_)){
        ROS_ERROR("NAVIGATION: could not read 'collision_distance_treshold' from rosparam!");
        exit(-1);
    }

    if (!nh_.getParam("/navigation/tolerance_speed_collision", toleranceSpeedCollision_)){
        ROS_ERROR("NAVIGATION: could not read 'collision_distance_treshold' from rosparam!");
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

    velSub_ = nh_.subscribe(velTopic_, 1, &Navigation::BasicNavigation::velCallback, this);
    playerSub_ = nh_.subscribe(playerTopic_, 1, &Navigation::BasicNavigation::playerCallback, this);
    towerRectangleSub_ = nh_.subscribe(towerRectangleTopic_, 1, &Navigation::BasicNavigation::towerRectangleCallback, this);

    velPub_ = nh_.advertise<geometry_msgs::Twist>(velPubTopic_, 1);
    markerPub_ = nh_.advertise<visualization_msgs::Marker>("target", 1);

    goalServiceClient_ = nh_.serviceClient<behavior_control::GoalService>("/planning/goal_service");
    ROS_WARN("Waiting for service");
    goalServiceClient_.waitForExistence();
    ROS_WARN("Service Ready!");
    behavior_control::GoalService goalSrv;

    goalSrv.request.parameter_id = 1;
    if(goalServiceClient_.call(goalSrv)){
        currentTarget_ = goalSrv.response.tower_id - 1;
        ROS_INFO("Current target tower: %d", goalSrv.response.tower_id);
    }
    else{
        ROS_ERROR("Failed to call goal service");
    }

    imminentCollision_ = false;
}

void Navigation::BasicNavigation::run(){
    geometry_msgs::Vector3 desiredVel;
    geometry_msgs::Vector3 outVel;
    updateCurrentPos();

    if(imminentCollision_){
        desiredVel.x = 0.0;
        desiredVel.y = 0.0;
        desiredVel.z = 0.0;

        outVel = decelerate();
    }
    else{
        desiredVel = generateDesriredVel();
        imminentCollision_ = monitorCollision();
        if( imminentCollision_ ){
            goalServiceClient_.waitForExistence();
            behavior_control::GoalService goalSrv;

            goalSrv.request.parameter_id = 1;
            if(goalServiceClient_.call(goalSrv)){
                currentTarget_ = goalSrv.response.tower_id - 1;
            }
            else{
                ROS_ERROR("Failed to call goal service");
            }
        }
        outVel = accelerate();
    }

    if(magnitude(outVel) < toleranceSpeed_){
        imminentCollision_ = false;
    }

    // ROS_INFO("x: %2f, y:%2f", outVel.x, outVel.y);
    cmd_.linear = outVel;
    publishTarget(towers_[currentTarget_]);
    velPub_.publish(cmd_);
}

geometry_msgs::Vector3 Navigation::BasicNavigation::generateDesriredVel(){
    geometry_msgs::Vector3 desiredVel;
    geometry_msgs::Point32 target = towers_[currentTarget_];

    desiredDirection_ = atan2(current_pos_.y - target.y, current_pos_.x - target.x);
    // ROS_INFO("Desired direction %2f", desiredDirection_);

    desiredDirection_ -= currentRotationWrtMap_;
    desiredDirection_ += M_PI;
    // ROS_INFO("Desired direction rotated %2f", desiredDirection_);

    desiredVel.x = maxSpeed_ * cos(desiredDirection_);
    desiredVel.y = maxSpeed_ * sin(desiredDirection_);
    desiredVel.z = 0;

    return desiredVel;
}

geometry_msgs::Vector3 Navigation::BasicNavigation::accelerate(){
    geometry_msgs::Vector3 smoothedVel;
    float smoothedSpeed;
    float currentSpeed = magnitude(currentVel_);

    smoothedSpeed = std::min(maxSpeed_, currentSpeed + acceleration_);

    smoothedVel.x = smoothedSpeed * cos(desiredDirection_);
    smoothedVel.y = smoothedSpeed * sin(desiredDirection_);
    smoothedVel.z = 0.0;

    return smoothedVel;
}

geometry_msgs::Vector3 Navigation::BasicNavigation::decelerate(){
    geometry_msgs::Vector3 smoothedVel;
    float smoothedSpeed;
    float currentSpeed = magnitude(currentVel_);

    smoothedSpeed = std::max(float(0.0), currentSpeed - acceleration_*3);

    smoothedVel.x = smoothedSpeed * cos(desiredDirection_);
    smoothedVel.y = smoothedSpeed * sin(desiredDirection_);
    smoothedVel.z = 0.0;

    return smoothedVel;
}

bool Navigation::BasicNavigation::monitorCollision(){
    if(playerDistance_ < collisionDistanceTreshold_ && magnitude(currentVel_) > toleranceSpeedCollision_){
        float deltaAlpha = fabs(playerDirection_ - desiredDirection_);

        if(deltaAlpha > M_PI){
            deltaAlpha = 2*M_PI - deltaAlpha;
        }

        return deltaAlpha < (toleranceDelta_ * (M_PI / 180.0));
    }
    return false;
}

void Navigation::BasicNavigation::updateCurrentPos(){
        tf::StampedTransform transform;
        try{
            listener_.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(10.0));
            listener_.lookupTransform("/map", "/base_link", ros::Time(0), transform);
            current_pos_.x = transform.getOrigin().getX();
            current_pos_.y = transform.getOrigin().getY();
            current_pos_.z = transform.getOrigin().getZ();


            double roll, pitch, yaw;
            tf::Quaternion quat = transform.getRotation();            
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            
            currentRotationWrtMap_ = yaw;
            // ROS_INFO("Current rotation: %.2f", current_rotation_wrt_map_);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
}

void Navigation::BasicNavigation::playerCallback(geometry_msgs::PointStamped position){
    playerDirection_ = atan2(position.point.y, position.point.x);
    playerDistance_ = sqrt( pow(position.point.x, 2) + pow(position.point.y, 2));

}

void Navigation::BasicNavigation::velCallback(geometry_msgs::Twist vel){
    currentVel_ = vel.linear;
}

void Navigation::BasicNavigation::towerRectangleCallback(const geometry_msgs::PolygonStamped& poly){
    std::vector<geometry_msgs::Point32> points;
    points.resize(num_towers_);
    for(int i=0; i<num_towers_; i++){
        points[i] = poly.polygon.points[i];
    }
    updateTowerPositions(points);
}

void Navigation::BasicNavigation::updateTowerPositions(std::vector<geometry_msgs::Point32> points){
    for(int i=0; i<points.size(); i++){
        geometry_msgs::Point32 point = points[i];
        int index = matchTowerIndex(point);
        towers_[index].x = point.x;
        towers_[index].y = point.y;
    }
}

int Navigation::BasicNavigation::matchTowerIndex(geometry_msgs::Point32 point){
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

void Navigation::BasicNavigation::publishTarget(geometry_msgs::Point32 target){
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

float Navigation::BasicNavigation::magnitude(geometry_msgs::Vector3 vector){
    return sqrt( pow(vector.x, 2) + pow(vector.y, 2) );
}