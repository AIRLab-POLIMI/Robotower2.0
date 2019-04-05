#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point32.h>
#include "std_msgs/Float32.h"

namespace player_tracking{

    class PlayerRobotDistance{
        private:
             ros::NodeHandle nh_;
             ros::Subscriber playerPositionSub_;
             ros::Subscriber robotPositionSub_;
             ros::Publisher playerRobotDistancePub_; 
             geometry_msgs::PointStamped playerPosition;
             geometry_msgs::PointStamped robotPosition;

            
        public:
             void playerPoseCallback(geometry_msgs::PointStamped playerPosition_);
 
             void robotPoseCallback(geometry_msgs::PointStamped robotPosition_);

             void publishDistance();
            
             PlayerRobotDistance();
    };
}