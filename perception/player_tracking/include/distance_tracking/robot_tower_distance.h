#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point32.h>
#include "std_msgs/Float32.h"
#include <player_tracker/TowerArray.h>
#include <player_tracker/Tower.h>

namespace player_tracking{

    class RobotTowerDistance{
        private:
            ros::NodeHandle nh_;
            ros::Subscriber robotPositionSub_;
            ros::Subscriber towerPositionSub_;
            ros::Publisher robotTowerDistancePub_; 
            geometry_msgs::Point32 robotPosition;
            geometry_msgs::Point towerPosition;
            std::vector<geometry_msgs::Point> towers;
            std_msgs::Float32 distance;

            
        public:
            void playerPoseCallback(geometry_msgs::PointStamped robotPosition);
 

             void publishDistance();
            
             RobotTowerDistance();
    };
}