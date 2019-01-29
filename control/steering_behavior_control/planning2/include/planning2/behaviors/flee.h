#include "planning2/behaviors/steering_behavior.h"

using LocomotionPlanning::Flee;

class Flee: public SteeringBehavior{
    public:
    Flee():SteeringBehavior(){
        name_ = "flee";
        code_ = FLEE_CODE;

        updateWeightsMap_.insert({"speed", 1.1});
        updateWeightsMap_.insert({"mass", 0.4});

    }

    bool targetReached(geometry_msgs::Point32 current_pos);
    		
	void updateTargetPos(geometry_msgs::Point32 newTargetPos);

	geometry_msgs::Vector3 calculate_desired_velocity(geometry_msgs::Point32 current_pos);
		
	geometry_msgs::Vector3 calculate_steering_force(geometry_msgs::Vector3 current_vel, geometry_msgs::Vector3 desired_vel);
	
	float evaluate(double player_model);

};