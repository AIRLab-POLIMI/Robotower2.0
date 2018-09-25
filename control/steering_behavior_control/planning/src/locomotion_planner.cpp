#include <ros/ros.h>
#include "planning/locomotion_planning.h"

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Twist.h>


LocomotionPlanning::LocomotionPlanner::LocomotionPlanner(){
    ROS_INFO("Creating locomotion planner...");
    
    if (!nh_.getParam("/planning/vel_topic_pub", vel_topic_)){
        ROS_ERROR("LOCOMOTION PLANNER: could not read 'vel_topic' from rosparam!");
        exit(-1);
    }

    if (!nh_.getParam("/planning/behavior_topic", steering_topic_)){
        ROS_ERROR("LOCOMOTION PLANNER: could not read 'steering_topic' from rosparam!");
        exit(-1);
    }
    steering_sub_ = nh_.subscribe(steering_topic_, 1, &LocomotionPlanning::LocomotionPlanner::steeringCallback, this);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>(vel_topic_, 1);
    current_steering_.behavior_code = -1;
    planning_ = false;
}

void LocomotionPlanning::LocomotionPlanner::steeringCallback(const planning::SteeringBehaviorEncoded& steering){
    if(steering.behavior_code != current_steering_.behavior_code){
        current_steering_ = steering;
        //current_behavior_ = steering_factory_.generateSteeringBehavior(current_steering_.behavior_code, current_steering_.target);
		current_behavior_ = steering_factory_.generateSteeringBehavior(current_steering_);        
		planning_ = true;
        vehicle_.resetParams();
    }
}


void LocomotionPlanning::LocomotionPlanner::updateLoop(){
    if(planning_){
        geometry_msgs::Point32 target;

        int behavior_code = current_steering_.behavior_code;
        target = current_steering_.target;

        // Uncomment to use with Triskar
        vehicle_.updateCurrentPos();
        
        //current_behavior_->setTarget(target);
        vehicle_.setSteeringBehavior(current_behavior_);
        
        geometry_msgs::Twist cmd = vehicle_.generateCommandVel();
        // Uncomment to use with Triskar
        geometry_msgs::Twist rotated_cmd = vehicle_.alignCommand(cmd);
        vel_pub_.publish(rotated_cmd);
        // vel_pub_.publish(cmd);

        geometry_msgs::Vector3 linear_command = cmd.linear;
                
        // ROS_INFO("Generated velocity x:%.2f, y:%.2f, z:%.2f", linear_command.x, linear_command.y, linear_command.z);
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "planning_locomotion_node");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    LocomotionPlanning::LocomotionPlanner planner;

    while(ros::ok()){
        planner.updateLoop();
        ros::spinOnce();
        rate.sleep();
    }
}
