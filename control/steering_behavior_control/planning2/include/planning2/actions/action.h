#ifndef INTENTION_H
#define INTENTION_H

#include "planning2/Action.h"
#include "planning2/Target.h"

#include <activity_monitor/Activity.h>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/LaserScan.h>

#include <string>

#define RUNNING 0
#define COMPLETED 1
#define ABORTED 2

#define CAPTURE_TOWER_CODE 0
#define DECEIVE_CODE 1
#define ESCAPE_CODE 2
#define WAIT_CODE 3

namespace ActionPlanning{

    class Action{
        protected:
            std::string actionName_;
            int intentionId_;
            int id_;
            int actionCode_;

            std::vector<planning2::Target> targets_;

            int statusCode_;

        public:

            int getIntentionId(){
                return intentionId_;
            }

            int getId(){
                return id_;
            }

            int getCode(){
                return actionCode_;
            }

            std::string getName(){
                return actionName_;
            }

            planning2::Action generateActionMessage(){
                planning2::Action msg;

                msg.id = id_;
                msg.intention_id = intentionId_;
                msg.action_code = actionCode_;
                msg.action_name = actionName_;
                msg.targets = targets_;

                return msg;
            }

            void start(int actionId, int intentionId){
                id_ = actionId;
                intentionId_ = intentionId;
                statusCode_ = RUNNING;
            }
            
            virtual void generateTargets(activity_monitor::Activity gameState, geometry_msgs::PointStamped playerPos, sensor_msgs::LaserScan scan) = 0;

    };

    class CaptureTower;
    class Deceive;
    class Escape;
    class Wait;
}
#endif