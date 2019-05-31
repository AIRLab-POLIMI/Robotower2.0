#include "distance_tracking/change_target.h"


class ChangeTarget{

    private:
        ros::NodeHandle nh_;
        ros::Subscriber playerTowerDistanceSub_;
        ros::Subscriber robotTowerDistanceSub_;
	    ros::Subscriber towerTargetSub_;
        ros::Publisher changeTargetPub_;
        std_msgs::Float32 playerTowerDistance;
    	std_msgs::Float32 robotTowerDistance;
        std_msgs::Int8 currentTarget;
        
    public:

    ChangeTarget(){
        playerTowerDistanceSub_ = nh_.subscribe("/player_robot_distance", 1, &ChangeTarget::playerTowerDistanceCallback, this);
		robotTowerDistanceSub_ = nh_.subscribe("/robot_tower_distance", 1, &ChangeTarget::playerTowerDistanceCallback, this);
	    //towerTargetSub_ = nh_.subscribe("/start_attack", 1, &ChangeTarget::towerTargetCallback, this);
        changeTargetPub_ = nh_.advertise<std_msgs::Float32>("/end_attack", 1);

    }

   	void playerTowerDistanceCallback(std_msgs::Float32 playerTowerDistance_){
         playerTowerDistance = playerTowerDistance_;
    }

    void robotTowerDistanceCallback(std_msgs::Float32 robotTowerDistance_){
         robotTowerDistance = robotTowerDistance_;
    }

    void towerTargetCallback(std_msgs::Int8 towerTarget_) {
		currentTarget = towerTarget_;
	}

    void publishDistance(){
         
      
    }
        

};

int main(int argc, char **argv) {
	ros::init(argc, argv, "change_target");

	ros::NodeHandle nh;
    ros::Rate r(20);
    ChangeTarget changeTargetPub_;
    while(ros::ok()){
        changeTargetPub_.publishDistance();
        ros::spinOnce();
        r.sleep();
    }
	return 0;
}