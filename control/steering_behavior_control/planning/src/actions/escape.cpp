#include "action/escape.h"
#include "planning/SteeringBehaviorEncoded.h"

planning::SteeringBehaviorEncoded Action::Escape::generateSteeringMsg(int behavior_index){

    planning::SteeringBehaviorEncoded msg;
    std::string name = meaningful_behaviors_[behavior_index]->getName();
    msg.behavior = name;
    msg.behavior_code = meaningful_behaviors_[behavior_index]->getCode();
    msg.target = target_;
    msg.tower_target = -1;
    msg.priority = 1.0;
    return msg;
}

void Action::Escape::generateTargetPoint(){
    // TODO: choose player position to evade from
}