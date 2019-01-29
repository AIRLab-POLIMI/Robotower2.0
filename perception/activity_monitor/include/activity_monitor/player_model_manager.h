#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>

#include <vector>

#include <steering_behavior/KinematicUpdate.h>

#include "activity_monitor/Activity.h"
#include "activity_monitor/PlayerModel.h"

namespace ModelingManager
{
    class PlayerModelManager{
        public:
            PlayerModelManager();
        private:
            ros::NodeHandle nh_;
            ros::Subscriber player_activity_sub_;    // Subscriber to player activity
            ros::Subscriber start_interaction_sub_;   // Subscirber to interaction start
            ros::Subscriber abort_interaction_sub_;   // Subscirber to interaction aborting
            ros::Subscriber reset_sub_;               // Subscriber for game reset
            ros::Subscriber press_time_sub_;
            ros::Subscriber kinematic_update_sub_;

            ros::Publisher model_pub_;       // Publisher of player model
            ros::Publisher trigger_pub_;
            tf::TransformListener listener_;

            activity_monitor::Activity last_activity_msg_;
            steering_behavior::KinematicUpdate last_kinematic_update_msg_;

            double starting_time_;
            double estimated_time_arrival_;

            bool is_monitoring_;
            int target_index_to_monitor_;
            int previous_index_to_monitor_;

            double tower_over_robot_attraction_;
            double cumulative_estimate_;
            double count_;
            double charging_time_;

            double average_led_per_press_;
            double total_led_per_press_;
            int count_press_;

            double average_risk_taken_;
            double total_risk_taken_;
            int count_risk_;

            std::vector<double> expertise_window_;
            int expertise_index_;
            double expertise_sum_;
            bool full_;


            void playerActivityCallback(activity_monitor::Activity activity);

            void startInteractionCallback(std_msgs::Int8 target_index);
            void abortInteractionCallback(std_msgs::Int8 message);

            // Calculate static attractive power of tower
            double calculateTowerAttraction(int tower);

            // Calculate attraction perturbation of tower due to robot movement
            double calculateRobotPerturbation(int tower);

            // Estimate player's descripting hyperparameter
            double estimateHyperparameter();

            // Update current parameters with new observation
            activity_monitor::PlayerModel updateModel(double observation);

            void publishTriggerMarker();

            void resetCallback(std_msgs::Bool resetting);

            void pressTimeCallback(std_msgs::Float64 press_time_msg);

            double estimateRisk();

            void kinematicUpdateCallback(steering_behavior::KinematicUpdate msg);


        public:
            // Main loop cycle
            void run();

    };

}