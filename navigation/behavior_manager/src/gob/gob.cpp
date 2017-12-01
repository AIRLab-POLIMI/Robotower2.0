#include "gob/gob.h"

gob::Goal::Goal(std::string goal_name, float insistence_value, float change_per_time) :
name(goal_name), value(insistence_value), change(change_per_time) {}


float gob::Goal::getChange(){
    return 0;
}

float gob::Goal::getDiscontentment(float newValue){
    return newValue * newValue;
}
    
float gob::Action::getGoalChange(std::string goal){
    return goal_change_LookupTable[goal];
}

float gob::Action::getDuration(){
    return 0;
}

void gob::Action::updateGoalChange(std::string goal, float newValue){
    goal_change_LookupTable[goal] = newValue;
}

float gob::calculateDiscontentment(Action * action, std::vector<Goal*> & goals){
    // Keep a running total
    float discontentment = 0;

    // Loop through each goal
    for (int i=0; i < goals.size(); i++){
        // Calculate the new value after the action
        float newValue = goals[i]->getValue() + action->getGoalChange(goals[i]->getName());

        // Calculate the change due to time alone
        newValue += action->getDuration() * goals[i]->getChange();

        // Get the discontentment of this value
        discontentment += goals[i]->getDiscontentment(newValue);
    }
    
    return discontentment;
}

gob::Action* gob::chooseAction(std::vector<Action*> & actions, std::vector<Goal*> & goals){
    // Go through each action, and calculate the discontentment
    Action* bestAction = actions[0];
    float bestValue = calculateDiscontentment(bestAction, goals);

    // Loop through action to update best action and value
    for (int i=0; i < actions.size(); i++){
        float thisValue = calculateDiscontentment(actions[i], goals);
        if (thisValue < bestValue){
            bestValue = thisValue;
            bestAction = actions[i];
        }
    }

    return bestAction;
}