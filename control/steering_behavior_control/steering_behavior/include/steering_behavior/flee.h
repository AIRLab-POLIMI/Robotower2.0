#include "steering_behavior/steering_behavior.h"
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/LaserScan.h>

using SteeringBehavior::Flee;

class Flee: public SteeringBehavior {
	private:
		float safety_distance_;
		sensor_msgs::LaserScan current_scan_;

	public:
		Flee():SteeringBehavior(){}
		Flee(geometry_msgs::Point32 target_):SteeringBehavior(target_){}
		Flee(std::vector<geometry_msgs::Point32> targets_):SteeringBehavior(targets_){}
		Flee(geometry_msgs::Point32 target_, sensor_msgs::LaserScan scan):SteeringBehavior(target_){
			current_scan_ = scan;
		}

		geometry_msgs::Vector3 calculate_desired_velocity(geometry_msgs::Point32 current_pos);
        geometry_msgs::Vector3 calculate_steering_force(geometry_msgs::Vector3 current_vel, geometry_msgs::Vector3 desired_vel);

		bool evaluateSafety(geometry_msgs::Point32 current_pos);
		float evaluate();

		std::string getName(){
			return "flee";
		}

		int getCode(){
			return 0;
		}

		bool updateTarget(sensor_msgs::LaserScan scan, geometry_msgs::Point32 current_pos);

		std::vector<float> getUpdateWeights(){
            std::vector<float> output;
            output.resize(2);
            output[0] = 1;
            output[1] = 1;
            return output;
        }
};
