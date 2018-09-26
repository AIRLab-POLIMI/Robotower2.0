#include "steering_behavior/steering_behavior.h"
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/LaserScan.h>

using SteeringBehavior::Deception;

class Deception: public SteeringBehavior {
	public:
		Deception():SteeringBehavior(){
		}
		Deception(geometry_msgs::Point32 target_):SteeringBehavior(target_){}
        Deception(std::vector<geometry_msgs::Point32> targets_):SteeringBehavior(targets_){}

		geometry_msgs::Vector3 calculate_desired_velocity(geometry_msgs::Point32 current_pos);
        geometry_msgs::Vector3 calculate_steering_force(geometry_msgs::Vector3 current_vel, geometry_msgs::Vector3 desired_vel);
		float evaluate();

		std::string getName(){
			return "deception";
		}

		int getCode(){
			return 4;
		}

		bool updateTarget(sensor_msgs::LaserScan scan, geometry_msgs::Point32 current_pos, float current_rotation_wrt_map);

		std::vector<float> getUpdateWeights();
};