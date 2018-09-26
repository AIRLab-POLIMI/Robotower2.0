#ifndef LOCOMOTION_PLANNING_H
#define LOCOMOTION_PLANNING_H

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3.h>
#include "planning/SteeringBehaviorEncoded.h"

#include <steering_behavior/vehicle_model.h>
#include <steering_behavior/steering_behavior.h>
#include <visualization_msgs/Marker.h>


namespace LocomotionPlanning{
     class SteeringFactory{
        public:
			SteeringBehavior::SteeringBehavior* generateSteeringBehavior(int steering_code, geometry_msgs::Point32 target);
			SteeringBehavior::SteeringBehavior* generateSteeringBehavior(int steering_code, std::vector<geometry_msgs::Point32> targets);
			SteeringBehavior::SteeringBehavior* generateSteeringBehavior(int steering_code);
			SteeringBehavior::SteeringBehavior* generateSteeringBehavior(planning::SteeringBehaviorEncoded msg);
	};

	 class LocomotionPlanner{
		 public:
		    LocomotionPlanner();

		    void updateLoop();

		private:
		    ros::NodeHandle nh_;
		    ros::Subscriber steering_sub_;
		    ros::Publisher vel_pub_;
			ros::Publisher marker_pub_;

		    planning::SteeringBehaviorEncoded current_steering_;
		    SteeringFactory steering_factory_;

		    VehicleModel::PointVehicle vehicle_;
		    SteeringBehavior::SteeringBehavior *current_behavior_;

		    std::string steering_topic_;
		    std::string vel_topic_;

		    bool planning_;

		    void steeringCallback(const planning::SteeringBehaviorEncoded& steering);
		    geometry_msgs::Point32 generateTarget();
			void publishTarget(geometry_msgs::Point32 target);
 };

}

#endif
