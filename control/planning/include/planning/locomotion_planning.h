#ifndef LOCOMOTION_PLANNING_H
#define LOCOMOTION_PLANNING_H

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3.h>
#include "planning/SteeringBehaviorEncoded.h"

#include <steering_behavior/vehicle_model.h>
#include <steering_behavior/steering_behavior.h>
#include <steering_behavior/arrival.h>
#include <steering_behavior/seek.h>


namespace LocomotionPlanning{

 class LocomotionPlanner{
     public:
        LocomotionPlanner();

        void updateLoop();

    private:
        ros::NodeHandle nh_;
        ros::Subscriber steering_sub_;
        ros::Publisher vel_pub_;

        planning::SteeringBehaviorEncoded current_steering_;
        SteeringBehavior::SteeringFactory steering_factory_;

        VehicleModel::PointVehicle vehicle_;
        SteeringBehavior::SteeringBehavior *current_behavior_;

        std::string steering_topic_;
        std::string vel_topic_;

        bool planning_;

        void steeringCallback(const planning::SteeringBehaviorEncoded& steering);
        geometry_msgs::Point32 generateTarget();
 };

}

#endif