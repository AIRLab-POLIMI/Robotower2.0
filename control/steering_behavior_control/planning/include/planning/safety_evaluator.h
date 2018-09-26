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
            float safety_treshold_flee_;
            float collision_angle_treshold_;
            float window_width_;
            float speed_thd_;

            float current_speed_;
            double player_distance_;

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
            // bool checkHeadingSector();
            bool checkSector(int idx);
            void velCallback(const geometry_msgs::Twist& vel);
            void playerCallback(const geometry_msgs::PointStamped& position);
            // bool checkLaserCone(int starting_index, int number_of_indexes);
            float calculateAngleDiff(float angle1, float angle2);
            float convertIndexToAngle(int idx);
			
			int getNumberOfScans(float theta);
			int convertAngleToIndex(float theta);
			int getNextIndex(int idx);
			int validateNegativeIndex(int idx);
			bool isWindowSafe(int starting_index, int number_of_indexes);
			bool isHeadingSectorSafe();
        
        public:
            SafetyEvaluator();
            bool evaluateSafety();
    };
}

#endif
