#ifndef SAFETY_H
#define SAFETY_H

#include <vector>
#include <string.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>

#include "planning/SafetyMsg.h"

namespace Safety
{
    class SafetyEvaluator{
        private:
            ros::NodeHandle nh_;
            ros::Subscriber vel_sub_;
            ros::Subscriber laser_sub_;
            ros::Subscriber player_angle_sub_;
            ros::Publisher safety_pub_;

            sensor_msgs::LaserScan current_scan_;

            std::vector<std::vector<float>> current_scan_segmented_; 

            planning::SafetyMsg safety_msg_;

            std::string vel_topic_;
            std::string player_angle_topic_;
            std::string safety_topic_;

            float current_angle_motion_;
            float current_angle_player_;
            float safety_treshold_;

            int number_of_sections_;
            // Width of sector in decgrees
            int section_angular_width_;
            
            int count_;
            int other_count_;
            

            int current_heading_sector_;
            int current_player_sector_;

            bool currently_safe_;

            void laserCallback(const sensor_msgs::LaserScan& scan);
            bool isCollidingDirection();
            bool checkAllScan();
            bool checkHeadingSector();
            void velCallback(const geometry_msgs::Twist& vel);
            void playerCallback(const geometry_msgs::PointStamped& position);
        
        public:
            SafetyEvaluator();
            bool evaluateSafety();
    };
}

#endif