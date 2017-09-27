#ifndef GOB_H
#define GOB_H

#include <vector>
#include <map>
#include <string>

namespace gob{

/* DATA STRUCTURES AND INTERFACES*/

class Goal{
private:

    std::string name;
    float change;   // the change per unit of time (utility involving time)
    float value;    // insistence value
public:

    Goal(std::string goal_name, float insistence_value, float change);

    /* Returns the amount of change that the goal normally
    experiences, per unit of time*/
    float getChange();

    /* The numerical value we are trying to minimize (the cost function)*/
    float getDiscontentment(float newValue);

    /* Getters and Setters*/
    void setValue(float newValue){
        value = newValue;
    }

    float getValue(){
        return value;
    }

    std::string getName(){
        return name;
    }
};

class Action{
private:

    /*A lookup table for saving the action goal change value*/
    std::map<std::string, float> goal_change_LookupTable;
    float duration;     // estimated time to complete action
    std::string name;
public:

    Action(std::string action_name):name(action_name), duration(0){}

    /*Returns change in goal insistence that
    carrying out the action would provide*/
    float getGoalChange(std::string goal);

    /*Returns the time it will take to complete the action*/
    float getDuration();

    /*Updates a goal change value */
    void updateGoalChange(std::string goal, float newValue);

    /* SETTERS AND GETTERS*/
    void updateDuration(float newValue){
        duration = newValue;
    }   

    std::string getName(){
        return name;
    }
};


/* UTILITY FUNCTIONS*/
float calculateDiscontentment(Action* action, std::vector<Goal*> & goals);
Action* chooseAction(std::vector<Action*> & actions, std::vector<Goal*> & goals);

} // end namespace
#endif  // GOB_H