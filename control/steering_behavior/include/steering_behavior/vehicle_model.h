#include<ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include "steering_behavior/steering_behavior.h"
#include <tf/transform_listener.h>

namespace VehicleModel
{
    class PointVehicle{
        protected:
            float mass_; // Constant to simulate mass of actor
            float max_force_;
            float max_speed_;
            float current_rotation_wrt_map_;
            geometry_msgs::Vector3 current_vel_;
            geometry_msgs::Point32 current_pos_;
            SteeringBehavior::SteeringBehavior *steering_behavior_;
            ros::NodeHandle nh_;
            ros::Subscriber pose_sub_;
            ros::Subscriber vel_sub_;
            tf::TransformListener listener;


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

            void setMaxForce(float force){
                max_force_ = force;
            }

            void setSteeringBehavior(SteeringBehavior::SteeringBehavior *steering_behavior){
                steering_behavior_ = steering_behavior;
            }
    };
}
