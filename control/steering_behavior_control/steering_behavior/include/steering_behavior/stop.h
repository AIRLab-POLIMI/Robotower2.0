#include "steering_behavior/steering_behavior.h"
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/LaserScan.h>

using SteeringBehavior::Stop;

class Stop: public SteeringBehavior {
	public:
		Stop():SteeringBehavior(){}
		Stop(geometry_msgs::Point32 target_):SteeringBehavior(target_){}

		geometry_msgs::Vector3 calculate_desired_velocity(geometry_msgs::Point32 current_pos);
        geometry_msgs::Vector3 calculate_steering_force(geometry_msgs::Vector3 current_vel, geometry_msgs::Vector3 desired_vel);

		float evaluate();

		std::string getName(){
			return "stop";
		}

		int getCode(){
			return 2;
		}

		bool updateTarget(sensor_msgs::LaserScan scan, geometry_msgs::Point32 current_pos){return false;}

		std::vector<float> getUpdateWeights(){
            std::vector<float> output;
            output.resize(2);
            output[0] = 1;
            output[1] = 1;
            return output;
        }
};
