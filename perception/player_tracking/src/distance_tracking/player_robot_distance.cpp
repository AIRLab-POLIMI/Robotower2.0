#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include "distance_tracking/player_robot_distance.h"


class PlayerRobotDistance{

    private:
        ros::NodeHandle nh_;
        ros::Subscriber playerPositionSub_;
        ros::Subscriber robotPositionSub_;
        ros::Publisher playerRobotDistancePub_; 
        geometry_msgs::PointStamped playerPosition;
        geometry_msgs::Point32 robotPosition;
        std_msgs::Float32 distance;
        
    public:

    PlayerRobotDistance(){
        playerPositionSub_ = nh_.subscribe("/player", 1, &PlayerRobotDistance::playerPoseCallback, this);
        robotPositionSub_ = nh_.subscribe("/robot_pose", 1, &PlayerRobotDistance::robotPoseCallback, this);
        playerRobotDistancePub_ = nh_.advertise<std_msgs::Float32>("player_robot_distance", 1);

    }

    void playerPoseCallback(geometry_msgs::PointStamped playerPosition_){
         playerPosition = playerPosition_;
    }

    void robotPoseCallback(geometry_msgs::Point32 robotPosition_){
         robotPosition = robotPosition_;
    }

    void publishDistance(){
        distance.data = sqrt(pow(2, static_cast<float>(playerPosition.point.x) - robotPosition.x) + 
                            pow(2, static_cast<float>(playerPosition.point.y) - robotPosition.y));
         if(distance.data > 0)
             playerRobotDistancePub_.publish(distance);
      
    }
        

};

int main(int argc, char **argv) {
	ros::init(argc, argv, "player_robot_distance");

	ros::NodeHandle nh;
    ros::Rate r(20);
    PlayerRobotDistance playerRobotDistancePub;
    while(ros::ok()){
        playerRobotDistancePub.publishDistance();
        ros::spinOnce();
        r.sleep();
    }
	return 0;
}