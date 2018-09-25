#include "steering_behavior/steering_behavior.h"
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/LaserScan.h>

using SteeringBehavior::Arrival;

class Arrival: public SteeringBehavior {
    private:
        float slowing_radius_;
        float max_speed_;

        // TODO REMOVE
        int silly_counter_;
	public:
        Arrival():SteeringBehavior(){
            slowing_radius_ = 2.0;
            max_speed_ = 1.0;

            //TODO REMOVE
            silly_counter_ = 10;
        }
		Arrival(geometry_msgs::Point32 target_):SteeringBehavior(target_){
            slowing_radius_ = 2.0;
            max_speed_ = 1.0;

            //TODO REMOVE
            silly_counter_ = 10;
        }

        Arrival(std::vector<geometry_msgs::Point32> targets_):SteeringBehavior(targets_){
            slowing_radius_ = 2.0;
            max_speed_ = 1.0;

            //TODO REMOVE
            silly_counter_ = 10;
        }

		geometry_msgs::Vector3 calculate_desired_velocity(geometry_msgs::Point32 current_pos);
        geometry_msgs::Vector3 calculate_steering_force(geometry_msgs::Vector3 current_vel, geometry_msgs::Vector3 desired_vel);

        float evaluate();

        std::string getName(){
            return "arrival";
        }

        int getCode(){
            return 3;
        }

        bool updateTarget(sensor_msgs::LaserScan scan, geometry_msgs::Point32 current_pos){
			return false;
		}

        std::vector<float> getUpdateWeights(){
            std::vector<float> output;
            output.resize(2);
            output[0] = 1;
            output[1] = 1;
            return output;
        }

};
