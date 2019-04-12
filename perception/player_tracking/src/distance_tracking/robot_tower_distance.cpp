#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point32.h>
#include "std_msgs/Float32.h"
#include "distance_tracking/robot_tower_distance.h"


class RobotTowerDistance{

    private:
        ros::NodeHandle nh_;
        ros::Subscriber robotPositionSub_;
        ros::Subscriber towerPositionSub_;
        ros::Publisher robotTowerDistancePub_; 
        geometry_msgs::PointStamped robotPosition;
        geometry_msgs::Point towerPosition;
        std::vector<geometry_msgs::Point> towers;
        std_msgs::Float32 distance;
        
    public:

    RobotTowerDistance(){
        robotPositionSub_ = nh_.subscribe("/robot_pos", 1, &RobotTowerDistance::robotPoseCallback, this);
        robotTowerDistancePub_ = nh_.advertise<std_msgs::Float32>("robot_tower_distance", 1);

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

    }

    void robotPoseCallback(geometry_msgs::PointStamped robotPosition_){
         robotPosition = robotPosition_;
    }

    void towerPoseCallback(player_tracker::TowerArray towers){
         towerPosition = towers.towers[2].position;

    }


    void publishDistance(){
        distance.data = sqrt(pow(2, static_cast<float>(robotPosition.point.x) - static_cast<float>(towerPosition.x)) + 
                            pow(2, static_cast<float>(robotPosition.point.y) - static_cast<float>(towerPosition.y)));
         if(distance.data > 0)
             robotTowerDistancePub_.publish(distance.data);
      
    }
        

};

int main(int argc, char **argv) {
	ros::init(argc, argv, "robot_tower_distance");

	ros::NodeHandle nh;
    ros::Rate r(20);
    RobotTowerDistance robotTowerDistancePub_;
    while(ros::ok()){
        robotTowerDistancePub_.publishDistance();
        ros::spinOnce();
        r.sleep();
    }
	return 0;
}