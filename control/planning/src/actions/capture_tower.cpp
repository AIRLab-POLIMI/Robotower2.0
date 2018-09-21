#include "action/capture_tower.h"
#include "planning/SteeringBehaviorEncoded.h"

planning::SteeringBehaviorEncoded Action::CaptureTower::generateSteeringMsg(int behavior_index){
    geometry_msgs::Point32 t;

    planning::SteeringBehaviorEncoded msg;
    std::string name = meaningful_behaviors_[behavior_index]->getName();
    msg.behavior = name;
    msg.behavior_code = meaningful_behaviors_[behavior_index]->getCode();
    msg.target = target_;
    msg.tower_target = tower_target_;
    msg.priority = 1.0;
    return msg;
}

void Action::CaptureTower::generateTargetPoint(){
    // TODO: choose the best tower to take down
}