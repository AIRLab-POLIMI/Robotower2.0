#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "time_tracking/player_tower_time.h"

class playerTowerTime{

    private:
        ros::NodeHandle nh_;
        ros::Subscriber playerTowerDistanceSub_;
        ros::Subscriber robotTowerDistanceSub_;
        std_msgs::Float32 playerTowerDistance;
        std_msgs::Float32  robotTowerDistance;
        std_msgs::Float32 previousPTDistance;
        std_msgs::Float32 startTime;
        std_msgs::Float32 endTime;
        bool trigger = false;
        ros::Publisher playerTowerTimePub_;
        

    public:

    playerTowerTime(){

        playerTowerDistanceSub_ = nh_.subscribe("/player_tower_distance", 1, &playerTowerTime::playerTowerDistanceCallback, this);
        robotTowerDistanceSub_ = nh_.subscribe("/robot_tower_distance", 1, &playerTowerTime::robotTowerDistanceCallback, this);
        playerTowerTimePub_ = nh_.advertise<std_msgs::Float32>("player_tower_time", 1);

    }

    void playerTowerDistanceCallback(std_msgs::Float32 playerTowerDistance_){
         playerTowerDistance = playerTowerDistance_;
    }

    void robotTowerDistanceCallback(std_msgs::Float32 robotTowerDistance_){
         robotTowerDistance = robotTowerDistance_;
    }


    void publishAtTowerTime(){

        if(playerTowerDistance.data - previousPTDistance.data > 2.0 && !trigger){
            startTime.data = ros::Time::now().toSec();
            trigger = true;
            playerTowerTimePub_.publish(startTime.data)
        }
        if(playerTowerDistance.data - previousPTDistance.data > 0.3 && trigger){
            endTime.data = ros::Time::now().toSec();
            trigger = false;
            playerTowerTimePub_.publish(endTime.data - startTime.data);
        }
        previousPTDistance = playerTowerDistance;
        
    }
};

    int main(int argc, char **argv) {
	ros::init(argc, argv, "player_tower_time");

	ros::NodeHandle nh;
    ros::Rate r(20);
    playerTowerTime playerTowerTimePub_;
    while(ros::ok()){
        playerTowerTimePub_.publishAtTowerTime();
        ros::spinOnce();
        r.sleep();
    }
	return 0;
}
