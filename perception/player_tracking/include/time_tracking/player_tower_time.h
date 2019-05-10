#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Time.h"

namespace player_tracking{

    class playerTowerTime{
        private:
        ros::NodeHandle nh_;
        ros::Subscriber playerTowerDistanceSub_;
        ros::Subscriber robotTowerDistanceSub_;
        ros::Subscriber clockSub_;        
        std_msgs::Float32 playerTowerDistance;
        std_msgs::Float32  robotTowerDistance;
        std_msgs::Float32 previousPTDistance;
        std_msgs::Float32 startTime;
        std_msgs::Float32 endTime;
        bool trigger = false;
        ros::Publisher playerTowerTimePub_;

            
        public:
            void playerTowerDistanceCallback(std_msgs::Float32 playerTowerDistance_);

            void robotTowerDistanceCallback(std_msgs::Float32 robotTowerDistance_);
 

             void publishAtTowerTime();
            
             playerTowerTime();
    };
}