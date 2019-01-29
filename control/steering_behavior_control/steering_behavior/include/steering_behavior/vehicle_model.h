#include<ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int8.h>

#include <activity_monitor/PlayerModel.h>
#include "steering_behavior/steering_behavior.h"

#define SPEED_KEY "speed"
#define MASS_KEY "mass"

namespace VehicleModel
{
    class PointVehicle{
        protected:

            std::vector<std::map<std::string, float>> difficulties_;
            float mass_; // Constant to simulate mass of actor

            float min_mass_;
            float base_mass_;
            float max_mass_;

            float min_force_;
            float base_force_;
            float max_force_;

            float min_speed_;
            float base_speed_;
            float max_speed_;

            float current_speed_;
            float current_mass_;
            float custom_mass_;
            float current_force_;

            float baseline_speed_;

            bool is_custom_;
            
            float current_rotation_wrt_map_;
            float deception_target_change_distance_;

            float slowing_radius_;
            geometry_msgs::Vector3 current_vel_;
            geometry_msgs::Point32 current_pos_;
            SteeringBehavior::SteeringBehavior *steering_behavior_;
            ros::NodeHandle nh_;
            ros::Subscriber pose_sub_;
            ros::Subscriber vel_sub_;
            ros::Subscriber scan_sub_;
            ros::Subscriber difficulty_sub_;

            ros::Publisher kinematic_update_pub_;

            sensor_msgs::LaserScan current_scan_;
            tf::TransformListener listener;

            void initDifficulties();
            void setDifficulty(int level);


        public:
            // Outputs the command vel according to point mass
            PointVehicle();

            geometry_msgs::Twist generateCommandVel();

            // Updates the current position of the robot
            void updateCurrentPos(const geometry_msgs::Pose& msg);
			void updateCurrentPos();
            geometry_msgs::Twist alignCommand(geometry_msgs::Twist cmd);
            // Updates current velocity of robot
            void updateCurrentVelocity(const geometry_msgs::Twist& msg);

            void laserCallback(const sensor_msgs::LaserScan& scan){
                current_scan_ = scan;
            }

            void setMaxForce(float force){
                max_force_ = force;
            }

            float getMaxSpeed(){
                return max_speed_;
            }

            float getSlowingRadius(){
                return slowing_radius_;
            }

            float getDeceptionChangeTargetDistance(){
                return deception_target_change_distance_;
            }

            void setSteeringBehavior(SteeringBehavior::SteeringBehavior *steering_behavior){
                steering_behavior_ = steering_behavior;
                initBehavior();
            }

            void updateKinematicProperties(activity_monitor::PlayerModel model);

			void changeParams(std::vector<float> update_weights);
            void resetParams();
            void initBehavior();

            void difficultyCallback(std_msgs::Int8 difficulty);
    };
}
