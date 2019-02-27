/** 
 *  @file    bt_control.h
 *  @author  Ewerton Lopes (ewerton.lopes@polimi.it)
 *  @date    04/30/2018  
 *  @version 1.0 
 *  
 *  @brief Implements Goal-Oriented Behavior for Decision Making
 *  using Behavior Tree as formalism and bayesian skill learning for
 *  fitting parameters.
 *
 *  @section DESCRIPTION
 *  
 *  This packages uses the ROS behavior_tree library
 *  in combination with the bayesian_skill_learning package
 *  for driving the robot's behavior. The general tree scheme
 *  implemented by this package is:
 *
 *                      ROOT
 *                       |
 *                       v
 *                     BEHAVIOR
 *                    (Fallback)
 *                    /        \
 *                   /          \
 *                  /            \
 *             EVALUATION        ATTACK
 *             (Sequence)    (Random Fallback)
 *               /   \             %   %    %  
 *              /     \            |  ...    \
 *      getMeanRisk  estimate     WEAK        HARD
 *        (action)    skill    (Sequence)  (Sequence)
 *                   (action)      |           |
 *                               REPEAT       ...
 *                           UNTIL FAILURE
 *                            (Decorator)
 *                                 |
 *                                LINK
 *                              /  |   \
 *                             /   |    \
 *                            /    |     \
 *                      getGoal   Move    ContinueOnMode
 *                    (Action)  (Action)    (condition)
 *
 *  , where the elipsis describe the repetition of the neighboring branches. Here,
 *  such repetition denotes the difficulty settings on which a tower attack can be
 *  performed under. The LINK node make it explicity the fact that we shall reuse
 *  that subtree since it is common for the difficulty settings.
 *  
 *  estimate skill (action), is the action that performs the bayesian skill learning.
 */

#ifndef BTCONTROL_H
#define BTCONTROL_H

#include <behavior_tree_core/action_node.h>
#include <behavior_tree_core/condition_node.h>
#include <string>

namespace Behavior{

class GetMeanRiskAction : public ActionNode{
public:
    explicit GetMeanRiskAction(std::vector<float>* risks);
    ~GetMeanRiskAction();
    void WaitForTick();
    void Halt();
private:
    std::vector<float>* risks
};

class PlannerAction : public ActionNode{
public:
    explicit PlannerAction();
    ~PlannerAction();
    void WaitForTick();
    void Halt();
private:
};

class ContinueOnModeCondition : public ConditionNode{
public:
    // Constructor
    explicit ContinueOnModeCondition(std::string Name);
    ~ContinueOnModeCondition();
    void set_boolean_value(bool boolean_value);

    BT::ReturnStatus Tick();
private:
    bool boolean_value_;
};

}  // namespace Behavior
#endif BTCONTROL_H