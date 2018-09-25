#include "action/deceive.h"
#include "planning/SteeringBehaviorEncoded.h"

planning::SteeringBehaviorEncoded Action::Deceive::generateSteeringMsg(int behavior_index){
    planning::SteeringBehaviorEncoded msg;
    std::string name = meaningful_behaviors_[behavior_index]->getName();
    msg.behavior = name;
    msg.behavior_code = meaningful_behaviors_[behavior_index]->getCode();
    msg.targets = targets_;
    msg.tower_target = 0;
    msg.priority = 1.0;
    return msg;
}

void Action::Deceive::generateTargetPoint(){
    // TODO: choose player position to evade from
}