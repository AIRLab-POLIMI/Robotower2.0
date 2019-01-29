#include "planning2/actions/action.h"
#include <behavior_control/Goal.h>
#include <ros/subscriber.h>
#include <ros/node_handle.h>

using ActionPlanning::CaptureTower;

class CaptureTower: public Action{
    private:
        ros::NodeHandle nh_;
        ros::Subscriber goalSub_;
        int currentGoal_;

    public:
        CaptureTower():Action(){
            actionCode_ = CAPTURE_TOWER_CODE;
            actionName_ = "capture_tower";

            goalSub_ = nh_.subscribe("/game/goal", 1, &ActionPlanning::CaptureTower::goalCallback, this);
            
        }

        void generateTargets(activity_monitor::Activity gameState, geometry_msgs::PointStamped playerPos, sensor_msgs::LaserScan scan);
        void goalCallback(behavior_control::Goal goalMsg);

};