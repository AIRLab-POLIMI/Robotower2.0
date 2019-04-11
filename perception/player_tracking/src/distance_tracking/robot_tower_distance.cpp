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
        geometry_msgs::Point32 towerPosition;
        std_msgs::Float32 distance;
        
    public:

    RobotTowerDistance(){
        robotPositionSub_ = nh_.subscribe("/robot_pos", 1, &RobotTowerDistance::robotPoseCallback, this);
        towerPositionSub_ = nh_.subscribe("/tower_pos", 1, &RobotTowerDistance::towerPoseCallback, this);
        robotTowerDistancePub_ = nh_.advertise<std_msgs::Float32>("robot_tower_distance", 1);

    }

    void robotPoseCallback(geometry_msgs::PointStamped robotPosition_){
         robotPosition = robotPosition_;
    }

    void towerPoseCallback(planning::TowerPositions positions){
         towerPosition = positions.towers[0];

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