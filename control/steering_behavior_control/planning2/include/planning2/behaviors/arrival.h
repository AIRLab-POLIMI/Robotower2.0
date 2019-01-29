#include "planning2/behaviors/steering_behavior.h"

using LocomotionPlanning::Arrival;

class Arrival: public SteeringBehavior{
    public:
    Arrival():SteeringBehavior(){
        name_ = "arrival";
        code_ = ARRIVAL_CODE;

        updateWeightsMap_.insert({"speed", 1.0});
        updateWeightsMap_.insert({"mass", 1.0});
    }

    bool targetReached(geometry_msgs::Point32 current_pos);
    		
	void updateTargetPos(geometry_msgs::Point32 newTargetPos);

	geometry_msgs::Vector3 calculate_desired_velocity(geometry_msgs::Point32 current_pos);
		
	geometry_msgs::Vector3 calculate_steering_force(geometry_msgs::Vector3 current_vel, geometry_msgs::Vector3 desired_vel);
	
	float evaluate(double player_model);

};