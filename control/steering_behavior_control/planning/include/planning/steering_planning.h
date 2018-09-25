#ifndef STEERING_PLANNING_H
#define STEERING_PLANNING_H

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3.h>
#include "planning/ActionEncoded.h"
#include "action/action_lib.h"


namespace SteeringPlanning{

 class SteeringPlanner{
    public:
        SteeringPlanner();

        void updateLoop();
    private:
        ros::NodeHandle nh_;
        ros::Subscriber action_sub_;
        ros::Publisher steering_pub_;

        planning::ActionEncoded current_action_msg;

        Action::ActionFactory action_factory_;
        Action::AbstractAction* current_action_;

        std::string action_topic_;
        std::string steering_topic_;

        bool intention_changed_;

        //TODO REMOVE
        bool swap_;

        void actionCallback(const planning::ActionEncoded& msg);
        int selectTower();
        int getBestBehaviorIndex(Action::AbstractAction* action);

 };

}

#endif