#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Point32.h"
#include <geometry_msgs/Twist.h>

namespace player_tracking{

    class cutPlayerOff {

    private:
        ros::NodeHandle nh_;
        ros::Subscriber playerTowerDistanceSub_;
        ros::Subscriber robotTowerDistanceSub_;
        ros::Subscriber robotBorderDistanceSub_;
        ros::Subscriber velocityControllerSub_;
        ros::Subscriber robotPositionSub_;
        ros::Subscriber towerTargetSub_;
        std_msgs::Float32 playerTowerDistance;
        std_msgs::Float32  robotTowerDistance;
        std_msgs::Float32 playerRobotDistance;
        std_msgs::Float32  robotBorderDistance;
        geometry_msgs::Point32 robotPosition;
        ros::Publisher velocityControllerPub_;
        geometry_msgs::Twist newTwistVel;
        float borders [4];
        std_msgs::Int8 previousTarget;
        std_msgs::Int8 currentTarget;

    public:

        cutPlayerOff();
        void playerRobotDistanceCallback(std_msgs::Float32 playerRobotDistance_);
        void robotPoseCallback(geometry_msgs::Point32 robotPosition_);
        void towerTargetCallback(std_msgs::Int8 towerTarget_);
        void robotBorderDistanceCallback(geometry_msgs::PolygonStamped towerPolygon);
        void velocityControllerCallback(geometry_msgs::Twist twistVel_);
        void checkCutOffConditions();
    };

}