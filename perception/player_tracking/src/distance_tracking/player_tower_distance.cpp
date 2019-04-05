#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include "planning/TowerPositions.h"
#include <geometry_msgs/Point32.h>
#include "std_msgs/Float32.h"
#include "distance_tracking/player_tower_distance.h"


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

    PlayerTowerDistance(){
        playerPositionSub_ = nh_.subscribe("/player", 1, &PlayerTowerDistance::playerPoseCallback, this);
        playerTower1DistancePub_ = nh_.advertise<std_msgs::Float32>("player_tower_distance", 1);

    }

    void playerPoseCallback(geometry_msgs::PointStamped playerPosition_){
         playerPosition = playerPosition_;
         tower1Position.x = 3.96;
        tower1Position.y = -4.76;
    }


    void publishDistance(){
        tower1Position.x = 3.96;
        tower1Position.y = -4.76;
        distance.data = sqrt(pow(2, static_cast<float>(playerPosition.point.x) - tower1Position.x) + 
                            pow(2, static_cast<float>(playerPosition.point.y) - tower1Position.y));
         if(distance.data > 0)
             playerTower1DistancePub_.publish(distance.data);
      
    }
        

};

int main(int argc, char **argv) {
	ros::init(argc, argv, "player_tower_distance");

	ros::NodeHandle nh;
    ros::Rate r(20);
    PlayerTowerDistance playerTower1DistancePub_;
    while(ros::ok()){
        playerTower1DistancePub_.publishDistance();
        ros::spinOnce();
        r.sleep();
    }
	return 0;
}