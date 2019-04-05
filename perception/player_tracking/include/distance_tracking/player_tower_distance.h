#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point32.h>
#include "std_msgs/Float32.h"

namespace player_tracking{

    class PlayerTowerDistance{
        private:
            ros::NodeHandle nh_;
            ros::Subscriber playerPositionSub_;
            ros::Subscriber towerPositionSub_;
            ros::Publisher playerTower1DistancePub_; 
            geometry_msgs::PointStamped playerPosition;
            geometry_msgs::Point32 tower1Position;
            std_msgs::Float32 distance;

            
        public:
             void playerPoseCallback(geometry_msgs::PointStamped playerPosition_);
 

             void publishDistance();
            
             PlayerTowerDistance();
    };
}