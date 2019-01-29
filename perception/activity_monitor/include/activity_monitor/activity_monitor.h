#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>
#include <tf/transform_listener.h>

#include <vector>

namespace ActivityMonitoring
{
    class ActivityMonitor{
        public:
            ActivityMonitor();
        private:
            ros::NodeHandle nh_;
            ros::Subscriber player_pos_sub_;
            ros::Subscriber target_monitor_sub_; // Subscriber to trigger the start of monitoring the motion of the player towards a given tower
            ros::Subscriber abort_monitor_sub_;  // Subscriber to stop the monitoring for safety issues
            ros::Subscriber tower_pos_sub_;      // Subscirber to tower position
            ros::Subscriber vel_sub_;            // Subscriber to robot's velocity

            ros::Publisher motion_wrt_robot_;  // Publishes the motion of the player with respect to the robot
            ros::Publisher motion_wrt_target_; // Publishes the motion of the player with respect to target tower

            std::vector<geometry_msgs::Point32> tower_pos_;

            bool is_monitoring_target_;
            int current_tower_target_;
            int closest_tower_index_;

            geometry_msgs::Point current_robot_pos_;
            geometry_msgs::Vector3 current_robot_vel_;
            double current_robot_speed_;
            double current_robot_rotation_wrt_map_;
            
            geometry_msgs::Point current_player_position_;
            double average_player_speed_;
            double current_robot_player_distance_;
            std::vector<double> current_player_tower_distance_;
            std::vector<double> current_robot_tower_distance_;
            std::vector<double> current_robot_tower_angles_;

            tf::TransformListener listener_;

            // Callback for player position wrt robot
            void playerPositionCallback(const geometry_msgs::PointStamped position);

            // Evaluate if the player position is stationary
            bool isStationary(geometry_msgs::Point new_pos);

            // Update distances between player and relevant positions
            void updateDistances(geometry_msgs::Point new_pos);

            // Evaluate wheter the player is going in the same direction of the robot or not
            int evaluateMotionWrtRobot(geometry_msgs::Point new_pos);

            // Evaluate which tower the player is going to
            int evaluatePlayerTargetTower();

            // Updates robot position wrt map
            void updateCurrentRobotPos();

            // Evaluate wheter the observation is outside the playground
            bool isOutOfBound();

            // Evaluate wheter the player is going to the robot target or not
            int evaluateMotionWrtTarget(geometry_msgs::Point new_pos);

            // Callback to start monitor motion wrt target
            void monitorTargetCallback(const std_msgs::Int8 target_index);
            
            // Callback to abort the monitoring of motion wrt target
            void abortMonitorTargetCallback(const std_msgs::Int8 target_index);  

            // Callback to robot's velocity
            void velCallback(const geometry_msgs::Twist vel);       

            // Function to express robot's velocity in map frame
            geometry_msgs::Vector3 rotateToMap(geometry_msgs::Vector3 vector);      

            // Transform point from robot frame to map frame
            geometry_msgs::Point transformPointToMap(geometry_msgs::Point in);


            // Function to estimate player model
            double calculateTowerAttraction(int tower_index);
            // Function to estimate player model
            double calculateRobotPerturbation(int tower_index);
            // Function to estimate player's hyperparameter
            double estimatePlayerHyperparameter();
    };

}
