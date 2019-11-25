#ifndef ACTION_PLANNING_H
#define ACTION_PLANNING_H

#include <ros/ros.h>
#include <vector>

#include <std_msgs/Bool.h>
#include <geometry_msgs/Point32.h>
#include "std_msgs/Float32.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#include "planning/SafetyMsg.h"
#include "planning/ActionEncoded.h"
#include <behavior_with_deception/DeceptionCommandMsg.h>
#include <behavior_with_deception/Deception.h>

namespace ActionPlanning{

 class ActionPlanner{
    public:
        ActionPlanner();
        void updateLoop();

    private:
        ros::NodeHandle nh_;
        ros::Subscriber safety_sub_;
        ros::Subscriber deception_command_sub_;
        ros::Subscriber playerRobotDistanceSub_;
		ros::Subscriber playerTowerDistanceSub_;
        ros::Subscriber robotTowerDistanceSub_;
        ros::Publisher action_pub_;

        std::string action_topic_;
        std::string safety_topic_;
        std::string deception_command_topic_;

        std_msgs::Float32 playerRobotDistance;
		std_msgs::Float32 playerTowerDistance;
        std_msgs::Float32 robotTowerDistance;
		ros::Time startTime;
		ros::Time endTime;
        ros::Time startNearTowerTime;
		ros::Time endNearTowerTime;
		std_msgs::Float32 interval;
		bool trigger;
        bool changeTrigger;
        double min_close_interval;
        double initial_close_interval;
		ros::Publisher intervalPub_;
		ros::Publisher changeDecisionPub_;

        planning::ActionEncoded last_action_;

        planning::SafetyMsg safety_msg_;
		behavior_with_deception::DeceptionCommandMsg deceiving_command_msg_;

        bool currently_being_deceptive_;
        bool is_first_deceptive_message_;

        int priority_;
        int safety_count_;

        void playerRobotDistanceCallback(std_msgs::Float32 playerRobotDistance_);
		void playerTowerDistanceCallback(std_msgs::Float32 playerTowerDistance_);
        void robotTowerDistanceCallback(std_msgs::Float32 robotTowerDistance_);

        void safetyCallback(const planning::SafetyMsg& msg);
		void deceptionCallback(const behavior_with_deception::Deception& msg);
        void deceptionCommandCallback(const behavior_with_deception::DeceptionCommandMsg& msg);
 };

}
#endif
