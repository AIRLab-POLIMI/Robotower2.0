#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Time.h"
#include <geometry_msgs/Twist.h>

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
        bool isPlayerFar = false;
        int counterFar = 0;
        geometry_msgs::Twist twistVel;
        geometry_msgs::Twist newTwistVel;

            
        public:
            void playerTowerDistanceCallback(std_msgs::Float32 playerTowerDistance_);

            void robotTowerDistanceCallback(std_msgs::Float32 robotTowerDistance_);
 

            void velocityControllerCallback(geometry_msgs::Twist twistVel_);

             void publishAtTowerTime();
            
             playerTowerTime();
    };
}