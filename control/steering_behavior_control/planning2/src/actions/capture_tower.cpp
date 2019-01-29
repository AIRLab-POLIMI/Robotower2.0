#include "planning2/actions/capture_tower.h"

void ActionPlanning::CaptureTower::generateTargets(activity_monitor::Activity gameState, geometry_msgs::PointStamped playerPos, sensor_msgs::LaserScan scan){
    // Evaluate which tower is the most appealing
    std::vector<planning2::Target> output;

    planning2::Target target;
    geometry_msgs::Point32 point;
    point.x = 1.0;
    point.y = 0.0;
    point.z = 0.0;

    target.target = point;
    target.is_tower = true;
    target.tower_index = currentGoal_;

    output.push_back(target);
    targets_ = output;
    currentGoal_ = 0;
}

void ActionPlanning::CaptureTower::goalCallback(behavior_control::Goal goalMsg){
    currentGoal_ = goalMsg.tower_number - 1;
}