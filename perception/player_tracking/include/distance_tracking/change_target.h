#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Int8.h"

namespace distance_tracking{

    class ChangeTarget{
        private:
        ros::NodeHandle nh_;
        ros::Subscriber playerTowerDistanceSub_;
        ros::Subscriber robotTowerDistanceSub_;
	    ros::Subscriber towerTargetSub_;
        ros::Publisher changeTargetPub_;
        std_msgs::Int8 currentTarget;
        

            
        public:
            void playerTowerDistanceCallback(std_msgs::Float32 playerTowerDistance_);

            void robotTowerDistanceCallback(std_msgs::Float32 robotTowerDistance_);
 
            void towerTargetCallback(std_msgs::Int8 towerTarget_);
        

             void publishDistance();
            
             ChangeTarget();
    };
}