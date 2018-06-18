/** 
 *  @file    gob.h
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

#ifndef GOB_H
#define GOB_H

#include <vector>
#include <map>
#include <string>

namespace gob{

class Goal{
private:

    std::string name;
    float change_per_time;
    float insistence;

public:

    Goal(std::string name, float insistence, float change_per_time);

    /**
    *   @brief Returns the amount of change that the goal normally
    *   experiences, per unit of time
    *   @return the change (float)
    */
    float getChange();

    /** 
    *   @brief  Returns the overal discontentment w.r.t the Goal.
    *   That is, the numerical value we are trying to minimize
    *   (the cost function) .   
    *  
    *   @param  new_value a value to be used (Normally, this 
    *   function applies some transformation to a given value,
    *   here we will use the square function)
    * 
    *   @return the discontentment (float)
    */  
    float getDiscontentment(float new_value);

    /**
    *   @brief a setter for the insistence value
    *   @param new_value new insistence
    *   @return Void
    */
    void setInsistence(float new_value);

    /**
    *   @brief a getter for the insistence value
    *   @return the insistence value (float)
    */
    float getInsistence();

    /**
    *   @brief a getter for the goal name
    *   @return the goal name (String)
    */
    std::string getName();
};

class Action{
private:

    std::string name;
    float duration;                                     // estimated time to complete action
    std::map<std::string, float> goal_change_table;     // A lookup table for saving 
                                                        // the action goal change insistence
    
public:

    Action(std::string action_name);

    /**
    *   @brief a setter for the goal_change value
    *   @param goal the goal name to update the goal_change for
    *   @param new_value new duration
    *   @return Void
    */
    void setGoalChange(std::string goal, float new_value);

   /**
    *   @brief a setter for the duration value
    *   @param new_value new duration
    *   @return Void
    */
    void setDuration(float new_value);

    /**
    *   @brief a getter for the action name
    *   @return the action name (String)
    */
    std::string getName();

    /**
    *   @brief Returns change in goal insistence that
    *          carrying out the action would provide
    *   @param goal the goal to get the insistence change for
    *   @return the change in goal insistence (float)
    */
    float getGoalChange(std::string goal);

    /*  */
    /**
    *   @brief Returns the estimated time it will take 
    *          to complete the action
    *   @return the duration (float)
    */
    float getDuration();
};


/**
*   @brief The calculateDiscontentment method returns the total 
*          discontentment associated with the state of the world, 
*          as given in the model
*   @param goals the vector of goals from which to calculate 
*                the discontentment
*   @param action the action to evaluate the discontentment
*   @return the discontentment value (float)
*/
float calculateDiscontentment(Action* action, std::vector<Goal*> &goals);

/**
*   @brief Loop through the actions and choose the one that produces the
*           least discontentment
*   @param goals the vector of goals from which to calculate 
*                the discontentment
*   @param action the vector of actions from which to choose the best from
*   @return the action pointer (Action pointer)
*/
Action* chooseAction(std::vector<Action*> & actions, std::vector<Goal*> & goals);

} // end namespace

#endif  // GOB_H