#include "steering_behavior/steering_behavior.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/LaserScan.h>

using SteeringBehavior::Flee;

class Flee: public SteeringBehavior {
	private:
		float safety_distance_;
		sensor_msgs::LaserScan current_scan_;
		bool to_update_;
		tf::TransformListener listener_;
		int randomized_center_index_;
		bool change_randomized_center_index_;
		
	public:
		Flee():SteeringBehavior(){}
		Flee(geometry_msgs::Point32 target_):SteeringBehavior(target_){
			to_update_ = true;
		}

		geometry_msgs::Vector3 calculate_desired_velocity(geometry_msgs::Point32 current_pos);
        geometry_msgs::Vector3 calculate_steering_force(geometry_msgs::Vector3 current_vel, geometry_msgs::Vector3 desired_vel);

		bool evaluateSafety(geometry_msgs::Point32 current_pos);
		float evaluate(double player_model);

		std::string getName(){
			return "flee";
		}

		int getCode(){
			return 0;
		}

		bool updateTarget(sensor_msgs::LaserScan scan, geometry_msgs::Point32 current_pos, float current_rotation_wrt_map);

		std::vector<float> getUpdateWeights();

		void updateTargetPos(std::vector<geometry_msgs::Point32> towers){
			return;
		}
};
