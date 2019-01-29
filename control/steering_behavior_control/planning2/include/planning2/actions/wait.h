#include "planning2/actions/action.h"

using ActionPlanning::Wait;

class Wait: public Action{

    public:
        Wait():Action(){
            actionCode_ = WAIT_CODE;
            actionName_ = "wait";
        }

        void generateTargets(activity_monitor::Activity gameState, geometry_msgs::PointStamped playerPos, sensor_msgs::LaserScan scan);

};