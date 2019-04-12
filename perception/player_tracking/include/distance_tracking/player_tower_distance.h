#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point32.h>
#include "std_msgs/Float32.h"
#include <player_tracker/TowerArray.h>
#include <player_tracker/Tower.h>

namespace player_tracking{

    class playerTowerDistance{
        private:
          ros::NodeHandle nh_;
          ros::Subscriber playerPositionSub_;
          ros::Subscriber towerPositionSub_;
          ros::Publisher playerTowerDistancePub_; 
          geometry_msgs::PointStamped playerPosition;
          geometry_msgs::Point towerPosition;
          std::vector<float> towers;
          std_msgs::Float32 distance;

            
        public:
            void playerPoseCallback(geometry_msgs::PointStamped playerPosition);
 

             void publishDistance();
            
             playerTowerDistance();
    };
}