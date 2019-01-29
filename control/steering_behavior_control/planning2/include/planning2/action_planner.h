#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PointStamped.h>
#include <planning2/Intention.h>

#include "planning2/actions/action.h"
#include <activity_monitor/Activity.h>


namespace ActionPlanning{

    class ActionPlanner{
        private:
            ros::NodeHandle nh_;
            ros::Subscriber intentionSub_;
            ros::Subscriber laserSub_;
            ros::Subscriber playerPosSub_;
            ros::Subscriber gameStateSub_;
            ros::Publisher actionPub_;

            std::map <int, std::vector<Action*>> actions_;

            Action* currentAction_;
            int currentActionId_;
            int currentIntentionId_;
            
            sensor_msgs::LaserScan currentScan_;
            activity_monitor::Activity currentGameState_;
            geometry_msgs::PointStamped currentPlayerPos_;

            void intentionCallback(planning2::Intention intention);
            void laserCallback(sensor_msgs::LaserScan scan);
            void gameStateCallback(activity_monitor::Activity gameState);
            void playerPosCallback(geometry_msgs::PointStamped playerPos);
            void initMap();

        public:
            ActionPlanner();
            void loop();


    };
}