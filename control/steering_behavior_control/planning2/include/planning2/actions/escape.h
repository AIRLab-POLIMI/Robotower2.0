#include "planning2/actions/action.h"
#include <tf/transform_listener.h>

using ActionPlanning::Escape;

class Escape: public Action{

    public:
        Escape():Action(){
            actionCode_ = ESCAPE_CODE;
            actionName_ = "escape";
        }

        void generateTargets(activity_monitor::Activity gameState, geometry_msgs::PointStamped playerPos, sensor_msgs::LaserScan scan);
    private:
        tf::TransformListener listener_;
        bool areAligned(float alpha1, float alpha2);

};