#ifndef LOCOMOTION_PLANNING_H
#define LOCOMOTION_PLANNING_H

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PolygonStamped.h>
#include "planning/SteeringBehaviorEncoded.h"

#include <activity_monitor/PlayerModel.h>
#include <steering_behavior/vehicle_model.h>
#include <steering_behavior/steering_behavior.h>
#include <visualization_msgs/Marker.h>


namespace LocomotionPlanning{
     class SteeringFactory{
        public:
			SteeringBehavior::SteeringBehavior* generateSteeringBehavior(planning::SteeringBehaviorEncoded msg);
	};

	 class LocomotionPlanner{
		 public:
		    LocomotionPlanner();

		    void updateLoop();

		private:
		    ros::NodeHandle nh_;
		    ros::Subscriber steering_sub_;
			ros::Subscriber tower_rectangle_sub_;
			ros::Subscriber player_model_sub_;
			ros::Subscriber reset_sub_;
		    ros::Publisher vel_pub_;
			ros::Publisher marker_pub_;
			ros::Publisher tower_pos_pub_;
			std::string tower_rectangle_topic_;

			std::vector<geometry_msgs::Point32> towers_;

		    planning::SteeringBehaviorEncoded current_steering_;
		    SteeringFactory steering_factory_;

		    VehicleModel::PointVehicle vehicle_;
		    SteeringBehavior::SteeringBehavior *current_behavior_;

		    std::string steering_topic_;
		    std::string vel_topic_;

		    bool planning_;

			double player_model_;

		    void steeringCallback(const planning::SteeringBehaviorEncoded& steering);
		    geometry_msgs::Point32 generateTarget();
			void publishTarget(geometry_msgs::Point32 target);

			void towerRectangleCallback(const geometry_msgs::PolygonStamped& poly);
			void updateTowerPositions(std::vector<geometry_msgs::Point32> points);
			int matchTowerIndex(geometry_msgs::Point32 point);
			void publishTowerPositions();
			void playerModelCallback(activity_monitor::PlayerModel model);
			void resetCallback(std_msgs::Bool reset);
 };

}

#endif
