#include "planning2/actions/escape.h"

void ActionPlanning::Escape::generateTargets(activity_monitor::Activity gameState, geometry_msgs::PointStamped playerPos, sensor_msgs::LaserScan scan){
    // Check if there's room to go to the center
    // if(playerDir - centerDir < thd) CANNOT GO TO CENTER
    std::vector<planning2::Target> output;
    planning2::Target target;
    geometry_msgs::Point32 point;
    tf::StampedTransform robotCenterTransform_;
    try{
        listener_.waitForTransform("/base_link", ros::Time(0), "/playground_center", ros::Time(0), "/map", ros::Duration(0.10));
        listener_.lookupTransform("/playground_center", "/base_link", ros::Time(0), robotCenterTransform_);
       
        float x_center = robotCenterTransform_.getOrigin().x(), y_center = robotCenterTransform_.getOrigin().y();
        float alpha_center = atan2(y_center, x_center);

        float x_player = playerPos.point.x, y_player = playerPos.point.y;
        float alpha_player = atan2(y_center, x_center);

        if(!areAligned(alpha_center, alpha_player)){
            // point.x = 1 * cos(alpha_center + M_PI) + gameState.robot_position_wrt_map.x;
            // point.y = 1 * sin(alpha_center + M_PI) + gameState.robot_position_wrt_map.y;
            point.x = gameState.robot_position_wrt_map.x + robotCenterTransform_.getOrigin().x();
            point.y = gameState.robot_position_wrt_map.y + robotCenterTransform_.getOrigin().y();
        }
        else{
            point.x = 1.0;
            point.y = 0.0;
            point.z = 0.0;
        }

    } catch (const std::exception& ex){
        ROS_WARN_STREAM(ex.what());
        point.x = 1.0;
        point.y = 0.0;
        point.z = 0.0;
    }
    // If it's not possible, look for a "free" direction
    target.target = point;
    target.is_tower = false;
    target.tower_index = -1;

    output.push_back(target);
    targets_ = output;
}

bool ActionPlanning::Escape::areAligned(float alpha1, float alpha2){
    return false;
}