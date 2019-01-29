#include "planning2/actions/wait.h"

void ActionPlanning::Wait::generateTargets(activity_monitor::Activity gameState, geometry_msgs::PointStamped playerPos, sensor_msgs::LaserScan scan){
    std::vector<planning2::Target> output;

    planning2::Target target;
    geometry_msgs::Point32 point;
    point.x = 0.0;
    point.y = 0.0;
    point.z = 0.0;

    target.target = point;
    target.is_tower = false;
    target.tower_index = -1;

    output.push_back(target);
    targets_ = output;
}