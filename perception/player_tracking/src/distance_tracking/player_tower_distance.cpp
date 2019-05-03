#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point32.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/Float32.h"
#include "distance_tracking/player_tower_distance.h"



class playerTowerDistance{

    private:
        ros::NodeHandle nh_;
        ros::Subscriber playerPositionSub_;
        ros::Subscriber towerPositionSub_;
        ros::Publisher playerTowerDistancePub_; 
        geometry_msgs::PointStamped playerPosition;
        geometry_msgs::Point towerPosition;
        std::vector<geometry_msgs::Point> towers;
        std_msgs::Float32 distance;
        
    public:

    playerTowerDistance(){
        playerPositionSub_ = nh_.subscribe("/player", 1, &playerTowerDistance::playerPoseCallback, this);
        towerPositionSub_ = nh_.subscribe("/target_steering", 1, &playerTowerDistance::towerPoseCallback, this);
        playerTowerDistancePub_ = nh_.advertise<std_msgs::Float32>("player_tower_distance", 1);
        /*
        for (int i=0; i < 4; i++){
        std::string str = "/tower_" + std::to_string(i+1);
        
        std::vector<float> tower_pos;
        if (!nh_.getParam(str, tower_pos)){
            ROS_FATAL_STREAM("Missing parameter: " << "/tower_" << std::to_string(i+1));
            exit(-1);
        }
        
        geometry_msgs::Point position;
        position.x = tower_pos[0];
        position.y = tower_pos[1];
        position.z = tower_pos[2]; 
        towers.push_back(position);
    }

        towerPosition = towers[2];
        */

    }

    void playerPoseCallback(geometry_msgs::PointStamped playerPosition_){
         playerPosition = playerPosition_;
    }

    void towerPoseCallback(visualization_msgs::Marker target){
         towerPosition.x = target.pose.position.x;
         towerPosition.y = target.pose.position.y;
    }



    void publishDistance(){
        distance.data = sqrt(pow(2, static_cast<float>(playerPosition.point.x) - static_cast<float>(towerPosition.x)) + 
                            pow(2, static_cast<float>(playerPosition.point.y) - static_cast<float>(towerPosition.y)));
         if(distance.data > 0)
             playerTowerDistancePub_.publish(distance.data);
      
    }
        

};

int main(int argc, char **argv) {
	ros::init(argc, argv, "player_tower_distance");

	ros::NodeHandle nh;
    ros::Rate r(20);
    playerTowerDistance playerTowerDistancePub_;
    while(ros::ok()){
        playerTowerDistancePub_.publishDistance();
        ros::spinOnce();
        r.sleep();
    }
	return 0;
}