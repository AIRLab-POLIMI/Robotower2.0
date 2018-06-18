/** 
 *  @file    gob.cpp
 *  @author  Ewerton Lopes (ewerton.lopes@polimi.it)
 *  @date    04/25/2018  
 *  @version 2.0 
 *  
 *  @brief Implements Goal-Oriented Behavior for Decision Making
 *
 *  @section DESCRIPTION
 *  
 *  This is a program that implements the Goal-Oriented Behavior
 *  for Decision Making in Robogame. We follow the technique 
 *  description in "Millington, Ian, and John Funge. Artificial
 *  intelligence for games. CRC Press, 2009."
 *  
 *  The robotic agent may have one or more goals, also called <motives>.
 *  Each goal has a level of importance (often called <insistence>)
 *  represented by a number. A goal with a high insistence will
 *  tend to influence the agent’s behavior more strongly.
 *
 *  The character will try to fulfill the goal or to reduce its 
 *  insistence. Some goals may be completely satisfied (such as "reaching 
 *  a position""). Others always have some positive insistence, and
 *  the agent simply reduce the insistence. A zero value for insistence
 *  is equivalent to a completely satisfied goal.
 *
 *  READ MORE in "Millington, Ian, and John Funge. Artificial
 *  intelligence for games. CRC Press, 2009."
 */

#include "gob/gob.h"

/* ----- GOAL CLASS DEFINITIONS ----- */

gob::Goal::Goal(std::string name, float insistence, float change_per_time) :
name(name), insistence(insistence), change_per_time(change_per_time) {}


float gob::Goal::getChange(){
    return 0;
}

float gob::Goal::getDiscontentment(float new_value){
    // Here we just square the new_value for having a non-linear transformation
    return new_value * new_value;
}

void gob::Goal::setInsistence(float new_value){
    insistence = new_value;
}

float gob::Goal::getInsistence(){
    return insistence;
}

std::string gob::Goal::getName(){
    return name;
}

/* ----- ACTION CLASS DEFINITIONS ----- */

gob::Action::Action(std::string action_name): name(action_name), duration(0){}

void gob::Action::setGoalChange(std::string goal, float new_value){
    goal_change_table[goal] = new_value;
}

void gob::Action::setDuration(float new_value){
    duration = new_value;
}

float gob::Action::getGoalChange(std::string goal){
    return goal_change_table[goal];
}

float gob::Action::getDuration(){
    return 0;
}

std::string gob::Action::getName(){
    return name;
}

/* ----- OTHER FUNCTION DEFINITIONS ----- */

float gob::calculateDiscontentment(Action * action, std::vector<Goal*> & goals){
    // Keep a running total
    float discontentment = 0;

    // Loop through each goal
    for (int i=0; i < goals.size(); i++){
        // Calculate the new value after the action
        float new_value = goals[i]->getInsistence() + action->getGoalChange(goals[i]->getName());

        // Calculate the change due to time alone
        new_value += action->getDuration() * goals[i]->getChange();

        // Get the discontentment of this value
        discontentment += goals[i]->getDiscontentment(new_value);
    }
    
    return discontentment;
}

gob::Action* gob::chooseAction(std::vector<Action*> & actions, std::vector<Goal*> & goals){
    // Go through each action, and calculate the discontentment
    Action* bestAction = actions[0];
    float bestValue = calculateDiscontentment(bestAction, goals);

    // Loop through action to update best action and value (least discontentment)
    for (int i=0; i < actions.size(); i++){
        float thisValue = calculateDiscontentment(actions[i], goals);
        if (thisValue < bestValue){
            bestValue = thisValue;
            bestAction = actions[i];
        }
    }

    return bestAction;
}