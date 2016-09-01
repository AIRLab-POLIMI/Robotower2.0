#ifndef ROSCONDITION_H
#define ROSCONDITION_H

#include <ConditionNode.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <robogame_bt_leaves/BTAction.h>

namespace BT
{
    class ROSCondition : public ConditionNode
    {
    public:
        // Constructor
        ROSCondition(std::string Name);
        ~ROSCondition();

        // The method that is going to be executed by the thread
        void Exec();
	  	robogame_bt_leaves::BTResult node_result;
        robogame_bt_leaves::BTGoal goal;
    };
}

#endif
