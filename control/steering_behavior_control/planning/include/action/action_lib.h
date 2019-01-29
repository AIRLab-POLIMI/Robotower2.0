#ifndef ACTION_LIBRARY_H
#define ACTION_LIBRARY_H

#include <steering_behavior/steering_behavior.h>
#include <vector>
#include <string.h>

#include <std_msgs/Int8.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
// #include <behavior_with_deception/Goal.h>
#include <behavior_control/Goal.h>


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
            // Subscriber to scan topic
            ros::Subscriber scan_sub_;
            // Subscriber to planner goal
            ros::Subscriber goal_sub_;
            ros::Subscriber parameter_sub_;
            ros::ServiceClient goal_service_client_;
            ros::Publisher start_interaction_pub_;
            ros::Publisher abort_interaction_pub_;
            ros::Publisher start_attack_pub_;
            ros::Publisher end_attack_pub_;
            tf::TransformListener listener;

            ros::NodeHandle nh_;

            // std::string tower_rectangle_topic_;
            // Vector containing intial position of towers
            std::vector<geometry_msgs::Point32> towers_;
            
            int current_goal_;
            int parameter_id_;
            geometry_msgs::Point32 current_pos_;
            sensor_msgs::LaserScan current_scan_;

            // int last_tower_index_;
            bool ready_;
            float current_rotation_wrt_map_;

            // Funciton to generate the intention to capture a tower. Tower number will be set by the goal topic, which this class is subscribed to
            Action::AbstractAction* generateCaptureAction();

            // Function to generate the intention to escape. The target from which escaping will be set analyzing the scan around the robot
            Action::AbstractAction* generateEscapeAction(planning::ActionEncoded action_msg);

            // Function to generate the intention to deceive. The targets are encoded inside the message
			Action::AbstractAction* generateDeceivingAction(planning::ActionEncoded action_msg);
            
            // Helper function to calculate temporary targets for deceiveing intention
            std::vector<geometry_msgs::Point32> generateDeceptiveTargets(geometry_msgs::Point32 real_target, geometry_msgs::Point32 fake_target);

            // Helper function to calculate temporary targets for deceiveing intention
            std::vector<geometry_msgs::Point32> generateDeceptiveTargetsBIS(geometry_msgs::Point32 real_target, geometry_msgs::Point32 deceptive_tower);
            
            void laserCallback(const sensor_msgs::LaserScan scan);
            // void goalCallback(const behavior_with_deception::Goal& goal_msg);
            void goalCallback(const behavior_control::Goal& goal_msg);
            void parameterCallback(const std_msgs::Int8 msg);
            void updateCurrentPos();

        public:
            ActionFactory();

            Action::AbstractAction* generateAction(planning::ActionEncoded action_msg);
            
    };
}
#endif
