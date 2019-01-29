#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>

#include "planning2/behaviors/steering_behavior.h"

namespace VehicleRepresentation{
    class PointVehicle{
        protected:
            ros::NodeHandle nh_;
            ros::Subscriber velSub_;

            float maxSpeed_;
            
            float currentMass_;
            float currentMaxSpeed_;
            float currentMaxForce_;

            std::map<std::string, float> updateWeightsMap_;


            geometry_msgs::Point32 currentPos_;
            geometry_msgs::Vector3 currentVel_;
            float currentRotationWrtMap_;

            LocomotionPlanning::SteeringBehavior* currentBehavior_;

            geometry_msgs::Twist alignWithMap(geometry_msgs::Twist cmd);

            void velCallback(geometry_msgs::Twist velocity);
            void initUpdateMap();

        public:
            PointVehicle();

            void setBehavior(LocomotionPlanning::SteeringBehavior* behavior);
            void locateVehicle(geometry_msgs::Point32 currentPos, float currentRotation);
            
            geometry_msgs::Twist generateCommandVel();

    };
}