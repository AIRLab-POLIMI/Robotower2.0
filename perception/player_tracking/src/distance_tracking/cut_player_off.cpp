#include "distance_tracking/cut_player_off.h"

class cutPlayerOff {

private:
	ros::NodeHandle nh_;
	ros::Subscriber playerRobotDistanceSub_;
	ros::Subscriber robotBorderDistanceSub_;
	ros::Subscriber velocityControllerSub_;
	ros::Subscriber robotPositionSub_;
	ros::Subscriber towerTargetSub_;
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

		playerRobotDistanceSub_ = nh_.subscribe("/player_robot_distance", 1, &cutPlayerOff::playerRobotDistanceCallback, this);
		robotBorderDistanceSub_ = nh_.subscribe("/tower_rectangle", 1, &cutPlayerOff::robotBorderDistanceCallback, this);
		robotPositionSub_ = nh_.subscribe("/robot_pose", 1, &cutPlayerOff::robotPoseCallback, this);
		towerTargetSub_ = nh_.subscribe("/start_attack", 1, &cutPlayerOff::towerTargetCallback, this);
		velocityControllerPub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	}

	void playerRobotDistanceCallback(std_msgs::Float32 playerRobotDistance_) {
	
		playerRobotDistance = playerRobotDistance_;
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
		
		if (playerRobotDistance.data > 0.7) {
			ROS_INFO("OOOOOOOOKKKKKKKKKKKKKKKKKKK PREVIOUS %d CURRENT %d ROBOTPOSE %f BORDER %f", currentTarget.data, previousTarget.data,
			robotPosition.x, borders[0]);
			if(currentTarget.data == 3 && previousTarget.data == 0 && robotPosition.x > borders[0]){
				newTwistVel.linear.x = -0.2;
           	    newTwistVel.linear.y = 0;
				
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