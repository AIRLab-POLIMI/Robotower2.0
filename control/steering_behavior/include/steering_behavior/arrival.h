#include "steering_behavior/steering_behavior.h"
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3.h>

using SteeringBehavior::Arrival;

class Arrival: public SteeringBehavior {
    private:
        float slowing_radius_;
        float max_speed_;
	public:
		Arrival(geometry_msgs::Point32 target_):SteeringBehavior(target_){
            slowing_radius_ = 2.0;
            max_speed_ = 1.0;
        }

		geometry_msgs::Vector3 calculate_desired_velocity(geometry_msgs::Point32 current_pos);
        geometry_msgs::Vector3 calculate_steering_force(geometry_msgs::Vector3 current_vel, geometry_msgs::Vector3 desired_vel);

};