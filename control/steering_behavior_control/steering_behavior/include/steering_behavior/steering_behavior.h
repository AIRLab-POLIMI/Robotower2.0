#ifndef STEERING_BEHAVIOR_H
#define STEERING_BEHAVIOR_H

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/LaserScan.h>

#include <cmath>
#include <vector>

#define FLEE 0
#define SEEK 1
#define STOP 2
#define ARRIVAL 3
#define DECEPTION 4

namespace SteeringBehavior{

 class SteeringBehavior{
	protected:
		geometry_msgs::Point32 target_;
		std::vector<geometry_msgs::Point32> targets_;
		int current_target_index_;

	public:
		SteeringBehavior(){}

		SteeringBehavior(geometry_msgs::Point32 target){
			target_ = target;
		}

		SteeringBehavior(std::vector<geometry_msgs::Point32> targets){
			current_target_index_ = 0;
			target_ = targets[current_target_index_];
			targets_ = targets;
		}

		geometry_msgs::Point32 getTarget(){
			return target_;
		}

		void setTarget(geometry_msgs::Point32 target){
			target_ = target;
		}

		void setTargets(std::vector<geometry_msgs::Point32> targets){
			targets_ = targets;
			current_target_index_ = 0;
			target_ = targets[current_target_index_]; 
		}

		virtual geometry_msgs::Vector3 calculate_steering_force(geometry_msgs::Vector3 current_vel, geometry_msgs::Vector3 desired_vel) = 0;

		// Calculates the velocity vector that would follow a straight path to the targert
		virtual geometry_msgs::Vector3 calculate_desired_velocity(geometry_msgs::Point32 current_pos) = 0;
		
		// Calculates the steering behavior score 
		virtual float evaluate() = 0;

		virtual std::string getName() = 0;
		virtual int getCode() = 0;
		virtual bool updateTarget(sensor_msgs::LaserScan scan, geometry_msgs::Point32 current_pos) = 0;
		virtual std::vector<float> getUpdateWeights() = 0;
	};

	class Flee;
	class Seek;
	class Stop;
	class Arrival;
	class Deception;

	// Static utility class to operate with vectors
	class VectorUtility{
		public:
			// Returns the vector representing the distance between two points
			static geometry_msgs::Vector3 vector_difference(geometry_msgs::Point32 point_1, geometry_msgs::Point32 point_2){
				geometry_msgs::Vector3 difference;
				difference.x = point_2.x - point_1.x;
				difference.y = point_2.y - point_1.y;
				difference.z = point_2.z - point_1.z;

				return difference;
			}

			// Returns the vector representing the distance between two vectors
			static geometry_msgs::Vector3 vector_difference(geometry_msgs::Vector3 vector_1, geometry_msgs::Vector3 vector_2){
				geometry_msgs::Vector3 difference;
				difference.x = vector_2.x - vector_1.x;
				difference.y = vector_2.y - vector_1.y;
				difference.z = vector_2.z - vector_1.z;

				return difference;
			}

			// Returns the vector representing the sum between two vectors
			static geometry_msgs::Vector3 vector_sum(geometry_msgs::Vector3 vector_1, geometry_msgs::Vector3 vector_2){
				geometry_msgs::Vector3 sum;
				sum.x = vector_2.x + vector_1.x;
				sum.y = vector_2.y + vector_1.y;
				sum.z = vector_2.z + vector_1.z;

				return sum;
			}

			// Returns the vector representing the sum between two vectors
			static geometry_msgs::Point32 vector_sum(geometry_msgs::Point32 vector_1, geometry_msgs::Vector3 vector_2){
				geometry_msgs::Point32 sum;
				sum.x = vector_2.x + vector_1.x;
				sum.y = vector_2.y + vector_1.y;
				sum.z = vector_2.z + vector_1.z;

				return sum;
			}

			// Returns the normalized vector
			static geometry_msgs::Vector3 normalize(geometry_msgs::Vector3 vector){
				geometry_msgs::Vector3 output;
				float magnitude = VectorUtility::magnitude(vector);
				output.x = vector.x / magnitude;
				output.y = vector.y / magnitude;
				output.z = vector.z / magnitude;

				return output;
			}

			// Returns the magnitude of the vector
			static float magnitude(geometry_msgs::Vector3 vector){
				float squared_sum = std::pow(vector.x,2) + std::pow(vector.y,2) + std::pow(vector.z,2);
				return std::pow(squared_sum, 0.5);
			}

			static geometry_msgs::Vector3 scalar_multiply(geometry_msgs::Vector3 vector, float scalar){
				geometry_msgs::Vector3 output;
				output.x = vector.x * scalar;
				output.y = vector.y * scalar;
				output.z = vector.z * scalar;

				return output;
			}

			// Truncates the input vector to the maximum magnitude allowed
			static geometry_msgs::Vector3 truncate(geometry_msgs::Vector3 vector, float max_magnitude){
				float magnitude = VectorUtility::magnitude(vector);

				if(magnitude > max_magnitude){
					// Truncate the force to the maximum allowed
					geometry_msgs::Vector3 output;
					output = VectorUtility::normalize(vector);
					output = VectorUtility::scalar_multiply(output, max_magnitude);
					return output;
				}
				// Return vector unchanged
				return vector;
			}

			// Returns the distance between two points
			static float distance(geometry_msgs::Point32 point_1, geometry_msgs::Point32 point_2){
				// TODO
				float squared_sum = pow(point_1.x - point_2.x, 2) + pow(point_1.y - point_2.y, 2);
				return sqrt(squared_sum);
			}

			static geometry_msgs::Vector3 revert(geometry_msgs::Vector3 vector){
				geometry_msgs::Vector3 output;

				output.x = -vector.x;
				output.y = -vector.y;
				output.z = -vector.z;
				return output;
			}
			
			static geometry_msgs::Vector3 rotate(geometry_msgs::Vector3 vector, float angle){
				geometry_msgs::Vector3 output;

				float current_orientation = atan2(vector.y, vector.x);
				float magnitude = VectorUtility::magnitude(vector);
				float new_orientation = current_orientation + angle;

				output.x = magnitude * cos(new_orientation);
				output.y = magnitude * sin(new_orientation);
				output.z = -vector.z;
				return output;
			}
	};

}

#endif
