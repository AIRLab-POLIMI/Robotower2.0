#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_listener.h>
#include <planning/TowerPositions.h>

#include <vector>

#include "activity_monitor/Activity.h"

namespace PlayerActivityMonitoring
{
    class PlayerActivityMonitor{
        public:
            PlayerActivityMonitor();
        private:
            ros::NodeHandle nh_;
            ros::Subscriber player_pos_sub_;    // Subscriber to player position
            ros::Subscriber tower_pos_sub_;     // Subscirber to tower position
            ros::Subscriber vel_sub_;           // Subscriber to robot velocity

            ros::Publisher activity_pub_;       // Publisher of activity recap
            ros::Publisher player_target_pub_;  // Publisher of marker for player target
            ros::Publisher risk_marker_pub_;
            tf::TransformListener listener_;

            activity_monitor::Activity activity_msg_;

            // Distance variables to be published
            double current_robot_player_distance_;
            std::vector<double> current_player_tower_distance_;
            std::vector<double> current_robot_tower_distance_;

            double current_robot_player_distance_variation_;
            std::vector<double> current_player_tower_distance_variation_;
            std::vector<double> current_robot_tower_distance_variation_;

            std::vector<double> robot_velocity_projections_;


            std::vector<geometry_msgs::Point32> tower_pos_; // Position of towers in the map frame

            
            geometry_msgs::Point current_player_position_;  // Position of the player in the map frame
            
            // Auxiliary variables to express the player position in the map frame
            geometry_msgs::Point current_robot_pos_;
            geometry_msgs::Vector3 current_robot_vel_;
            double current_robot_speed_;
            double current_robot_rotation_wrt_map_;            

            // Auxiliary variables to determine which tower the player is targeting
            int closest_tower_index_;
            std::vector<double> player_tower_angles_;

            // Callback for player position wrt robot
            void playerPositionCallback(const geometry_msgs::PointStamped position);

            // Callback for tower positions
            // void towerPosCallback(const xxx::yyy);

            // Publish message
            void publishActivityMsg();

            // Evaluate if the player position is stationary
            bool isStationary(geometry_msgs::Point new_pos);

            // Update distances between player and relevant positions
            void updateDistances(geometry_msgs::Point new_pos);

            // Update velocity projections
            void updateVelocityProjections();

            // Evaluate which tower the player is going to
            int evaluatePlayerTargetTower(geometry_msgs::Point new_pos);

            // Gets the closest tower from the player
            int getClosestTowerIndex();

            // Gets the index of the tower most at risk
            int getMostRiskyTower();

            // Updates robot position wrt map
            void updateCurrentRobotPos();

            // Evaluate wheter the observation is outside the playground
            bool isOutOfBound();   

            geometry_msgs::Vector3 rotateToMap(geometry_msgs::Vector3 vector);    

            void velCallback(const geometry_msgs::Twist vel);

            // Transform point from robot frame to map frame
            geometry_msgs::Point transformPointToMap(geometry_msgs::PointStamped in);

            // Callback to update tower positions
            void towerPosCallback(const planning::TowerPositions);

            // Publisher for visualization help
            void publishPlayerTargetMarker();
            void publishRiskyTower();
    };

}