#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "time_tracking/player_tower_time.h"

class playerTowerTime{

    private:
        ros::NodeHandle nh_;
        ros::Subscriber playerTowerDistanceSub_;
        ros::Subscriber robotTowerDistanceSub_;
        ros::Subscriber velocityControllerSub_;
        std_msgs::Float32 playerTowerDistance;
        std_msgs::Float32  robotTowerDistance;
        std_msgs::Float32 previousPTDistance;
        ros::Time startTime;
        ros::Time endTime;
        std_msgs::Float32 interval;
        bool trigger = false;
        ros::Publisher playerTowerTimePub_;
        ros::Publisher velocityControllerPub_;
        bool isPlayerFar = false;
        int counterFar = 0;
        geometry_msgs::Twist twistVel;
        geometry_msgs::Twist newTwistVel;
        double speedUp = 0.10;
        double maxSpeedUp = 0.14; 
        int timesAtTowerLimit = 3;  
        double rangeAtTower = 2.4;    

    public:

    playerTowerTime(){

        playerTowerDistanceSub_ = nh_.subscribe("/player_tower_distance", 1, &playerTowerTime::playerTowerDistanceCallback, this);
        robotTowerDistanceSub_ = nh_.subscribe("/robot_tower_distance", 1, &playerTowerTime::robotTowerDistanceCallback, this);
        velocityControllerSub_ = nh_.subscribe("/cmd_vel", 1, &playerTowerTime::velocityControllerCallback, this);
        playerTowerTimePub_ = nh_.advertise<std_msgs::Float32>("player_tower_time", 1);
        velocityControllerPub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        previousPTDistance = playerTowerDistance;
    }

    void playerTowerDistanceCallback(std_msgs::Float32 playerTowerDistance_){
         playerTowerDistance = playerTowerDistance_;
    }

    void robotTowerDistanceCallback(std_msgs::Float32 robotTowerDistance_){
         robotTowerDistance = robotTowerDistance_;
    }

    void velocityControllerCallback(geometry_msgs::Twist twistVel_){
         twistVel = twistVel_;
    }



    /*When a player stays to conquer a tower, publish a message with the time he actually stay fixed at the tower*/

    void publishAtTowerTime(){
        if(!trigger){
            if(playerTowerDistance.data - previousPTDistance.data > 1.0){
            startTime = ros::Time::now();
            trigger = true;
            }
        }
       
        else{
            if(previousPTDistance.data - playerTowerDistance.data > 0.3){
                endTime = ros::Time::now();
                trigger = false;
                interval.data = endTime.toSec() - startTime.toSec();
                playerTowerTimePub_.publish(interval);
                if(interval.data > rangeAtTower)
                    counterFar++; // counter to activate the trigger to speed up the robot
                if(counterFar >= timesAtTowerLimit)
                    isPlayerFar = true;

            }
        }

        previousPTDistance = playerTowerDistance;

        if(playerTowerDistance.data - robotTowerDistance.data > 1.25 && isPlayerFar)
            speedUpRobot();

    }



    /* Directly publish on cmd_vel to speed up the robot when the player stays at another tower*/

    void speedUpRobot(){

        if(twistVel.linear.x < 0 && twistVel.linear.y < 0){
            newTwistVel.linear.x = twistVel.linear.x - speedUp;
            newTwistVel.linear.y = twistVel.linear.y - speedUp;
            velocityControllerPub_.publish(newTwistVel);
        }
        else if(twistVel.linear.x > 0 && twistVel.linear.y < 0){
            newTwistVel.linear.x = twistVel.linear.x + speedUp;
            newTwistVel.linear.y = twistVel.linear.y - speedUp;
            velocityControllerPub_.publish(newTwistVel);
        }
        else if(twistVel.linear.x < 0 && twistVel.linear.y > 0){
            newTwistVel.linear.x = twistVel.linear.x - speedUp;
            newTwistVel.linear.y = twistVel.linear.y + speedUp;
            velocityControllerPub_.publish(newTwistVel);
        }
        else if(twistVel.linear.x > 0 && twistVel.linear.y > 0){
            newTwistVel.linear.x = twistVel.linear.x + speedUp;
            newTwistVel.linear.y = twistVel.linear.y + speedUp;
            velocityControllerPub_.publish(newTwistVel);
        }
        if(speedUp < maxSpeedUp)
            speedUp += 0.02;
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
