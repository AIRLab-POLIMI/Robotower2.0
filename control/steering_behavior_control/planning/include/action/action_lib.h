#ifndef ACTION_LIBRARY_H
#define ACTION_LIBRARY_H

#include <steering_behavior/steering_behavior.h>
#include <vector>
#include <string.h>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PolygonStamped.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>


#include <ros/ros.h>

#include "planning/ActionEncoded.h"
#include "planning/SteeringBehaviorEncoded.h"


namespace Action
{
    class AbstractAction{
        protected:
            std::vector<SteeringBehavior::SteeringBehavior*> meaningful_behaviors_;
            bool is_completed;
            bool is_aborted;

        public:
            AbstractAction(){}
            std::vector<SteeringBehavior::SteeringBehavior*> getMeaningfulBehaviors(){
                return meaningful_behaviors_;
            }

            bool isCompleted(){
                return is_completed;
            }

            bool isAborted(){
                return is_aborted;
            }

            virtual planning::SteeringBehaviorEncoded generateSteeringMsg(int behavior_index) = 0;
            virtual void generateTargetPoint() = 0;
            
    };

    class CaptureTower;
    class Escape;
	class Deceive;

    class ActionFactory{
        private:
            ros::Subscriber tower_pos_sub_;
            ros::Subscriber pose_sub_;
            ros::Subscriber scan_sub_;
            ros::Subscriber tower_rectangle_sub_;
            tf::TransformListener listener;

            ros::NodeHandle nh_;

            std::string tower_topic_;
            std::string pose_topic_;
            std::string tower_rectangle_topic_;
            std::vector<geometry_msgs::Point32> towers_;
            

            geometry_msgs::Point32 current_pos_;
            sensor_msgs::LaserScan current_scan_;

            int last_tower_index_;
            bool ready_;
            float current_rotation_wrt_map_;

            Action::AbstractAction* generateCaptureAction(int tower_index);
            Action::AbstractAction* generateEscapeAction(planning::ActionEncoded action_msg);
			Action::AbstractAction* generateDeceivingAction(planning::ActionEncoded action_msg);
            // void towerPositionCallback(const triskar_sim::TowerPositionArray positions);
            std::vector<geometry_msgs::Point32> generateDeceptiveTargets(geometry_msgs::Point32 real_target, geometry_msgs::Point32 fake_target);
            std::vector<geometry_msgs::Point32> generateDeceptiveTargetsBIS(geometry_msgs::Point32 real_target, geometry_msgs::Point32 deceptive_tower);
            void laserCallback(const sensor_msgs::LaserScan scan);
            void updateCurrentPos(const geometry_msgs::Pose& msg);
            void towerRectangleCallback(const geometry_msgs::PolygonStamped& msg);
            void updateTowerPositions(std::vector<geometry_msgs::Point32>);
            int matchTowerIndex(geometry_msgs::Point32 point);
            void updateCurrentPos();
            // void publishTarget(geometry_msgs::Point32 target);
        public:
            ActionFactory();

            Action::AbstractAction* generateAction(planning::ActionEncoded action_msg, int tower_index);
            
    };
}
#endif
