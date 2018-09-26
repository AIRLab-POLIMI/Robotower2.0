#include "steering_behavior/steering_behavior.h"
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/LaserScan.h>

using SteeringBehavior::Flee;

class Flee: public SteeringBehavior {
	private:
		float safety_distance_;
		sensor_msgs::LaserScan current_scan_;
		bool to_update_;

	public:
		Flee():SteeringBehavior(){}
		Flee(geometry_msgs::Point32 target_):SteeringBehavior(target_){
			to_update_ = true;
		}
		Flee(std::vector<geometry_msgs::Point32> targets_):SteeringBehavior(targets_){
			to_update_ = true;
		}
		Flee(geometry_msgs::Point32 target_, sensor_msgs::LaserScan scan):SteeringBehavior(target_){
			current_scan_ = scan;
			to_update_ = true;
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

		bool updateTarget(sensor_msgs::LaserScan scan, geometry_msgs::Point32 current_pos, float current_rotation_wrt_map);

		std::vector<float> getUpdateWeights(){
            std::vector<float> output;

			output.resize(3);
			output[0] = 2.0;
			output[1] = 2.0;
			output[2] = 2.0;
			
            return output;
        }
};
