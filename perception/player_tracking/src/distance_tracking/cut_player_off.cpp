#include "distance_tracking/cut_player_off.h"

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
	std_msgs::Float32 robotTowerDistance;
	std_msgs::Float32 playerRobotDistance;
	std_msgs::Float32  robotBorderDistance;
	geometry_msgs::Point32 robotPosition;
	ros::Publisher velocityControllerPub_;
	geometry_msgs::Twist newTwistVel;
	float borders [4];
	std_msgs::Int8 previousTarget;
	std_msgs::Int8 currentTarget;

public:

	cutPlayerOff() {

		playerTowerDistanceSub_ = nh_.subscribe("/player_tower_distance", 1, &cutPlayerOff::playerTowerDistanceCallback, this);
		robotTowerDistanceSub_ = nh_.subscribe("/robot_tower_distance", 1, &cutPlayerOff::playerTowerDistanceCallback, this);
		robotBorderDistanceSub_ = nh_.subscribe("/tower_rectangle", 1, &cutPlayerOff::robotBorderDistanceCallback, this);
		robotPositionSub_ = nh_.subscribe("/robot_pose", 1, &cutPlayerOff::robotPoseCallback, this);
		towerTargetSub_ = nh_.subscribe("/start_attack", 1, &cutPlayerOff::towerTargetCallback, this);
		velocityControllerPub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	}

	void playerTowerDistanceCallback(std_msgs::Float32 playerTowerDistance_){
         playerTowerDistance = playerTowerDistance_;
    }

    void robotTowerDistanceCallback(std_msgs::Float32 robotTowerDistance_){
         robotTowerDistance = robotTowerDistance_;
    }

	void robotPoseCallback(geometry_msgs::Point32 robotPosition_) {
		robotPosition = robotPosition_;
	}

	void towerTargetCallback(std_msgs::Int8 towerTarget_) {
		previousTarget = currentTarget;
		currentTarget = towerTarget_;
	}

	void robotBorderDistanceCallback(geometry_msgs::PolygonStamped towerPolygon) {
		
		/*borders constructor*/

		/*tower id
		1	2
		0	3
		
		cmd vel
		
		x + su
		x - giu
		y + sx
		y - dx*/
		
		if (towerPolygon.polygon.points[0].x < towerPolygon.polygon.points[3].x)
			borders[0] = towerPolygon.polygon.points[0].x;
		else
			borders[0] = towerPolygon.polygon.points[3].x;

		if (towerPolygon.polygon.points[0].y < towerPolygon.polygon.points[1].y)
			borders[1] = towerPolygon.polygon.points[0].y;
		else
			borders[1] = towerPolygon.polygon.points[1].y;

		if (towerPolygon.polygon.points[1].x < towerPolygon.polygon.points[2].x)
			borders[2] = towerPolygon.polygon.points[1].x;
		else
			borders[2] = towerPolygon.polygon.points[2].x;

		if (towerPolygon.polygon.points[2].y < towerPolygon.polygon.points[3].y)
			borders[3] = towerPolygon.polygon.points[2].y;
		else
			borders[3] = towerPolygon.polygon.points[3].y;


	}




	/*Compute when the robot can safely cut the player off*/

	void checkCutOffConditions() {
		playerRobotDistance.data = playerTowerDistance.data - robotTowerDistance.data;
		if (playerRobotDistance.data > 1.7) {
			if(((currentTarget.data == 3 && previousTarget.data == 0) || (currentTarget.data == 0 && previousTarget.data == 3)) 
						&& robotPosition.x > borders[0]){
				newTwistVel.linear.x = -0.2;
           	    newTwistVel.linear.y = 0;
                velocityControllerPub_.publish(newTwistVel);
			}

			if(((currentTarget.data == 3 && previousTarget.data == 2) || (currentTarget.data == 2 && previousTarget.data == 3)) 
						&& robotPosition.y > borders[3]){
				newTwistVel.linear.x = 0;
           	    newTwistVel.linear.y = -0.2;
				
                velocityControllerPub_.publish(newTwistVel);
			}
			if(((currentTarget.data == 1 && previousTarget.data == 2) || (currentTarget.data == 2 && previousTarget.data == 1)) 
						&& robotPosition.x > borders[2]){
				newTwistVel.linear.x = 0.2;
           	    newTwistVel.linear.y = 0;
				
                velocityControllerPub_.publish(newTwistVel);
			}
			if(((currentTarget.data == 1 && previousTarget.data == 0) || (currentTarget.data == 0 && previousTarget.data == 1))
			 			&& robotPosition.y > borders[1]){
				newTwistVel.linear.x = 0;
           	    newTwistVel.linear.y = 0.2;
				
                velocityControllerPub_.publish(newTwistVel);
			}

		}

	}


};

int main(int argc, char** argv) {
	ros::init(argc, argv, "cut_player_off");

	ros::NodeHandle nh;
	ros::Rate r(20);

	cutPlayerOff cutPlayerOffPub_;
	while (ros::ok()) {
		cutPlayerOffPub_.checkCutOffConditions();
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}