#ifndef ACTION_PLANNING_H
#define ACTION_PLANNING_H

#include <ros/ros.h>
#include <vector>

#include <std_msgs/Bool.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#include "planning/SafetyMsg.h"

namespace ActionPlanning{

 class ActionPlanner{
    public:
        ActionPlanner();

        //TODO
        int silly_counter_;
        int silly_priority_;

        void updateLoop();

    private:
        ros::NodeHandle nh_;
        ros::Subscriber safety_sub_;
		ros::Subscriber deception_sub_;
        ros::Publisher action_pub_;
        
        std::vector<std::vector<float>> current_scan_segmented_; 

        std::string action_topic_;
        std::string vel_topic_;
        std::string player_angle_topic_;
        std::string safety_topic_;
        std::string deception_topic_;

        planning::SafetyMsg safety_msg_;
		std_msgs::Bool deceiving_msg_;

        float current_angle_motion_;
        float last_know_player_position_;

        void safetyCallback(const planning::SafetyMsg& msg);
		void deceptionCallback(const std_msgs::Bool& msg);
 };

}
#endif
