#include <ros/ros.h>
#include <planning2/Action.h>

#include "planning2/behaviors/steering_behavior.h"
#include "planning2/vehicles/vehicle.h"
#include "planning2/actions/action.h"
#include "planning2/Target.h"
#include "planning2/TowerArray.h"

#include <activity_monitor/Activity.h>


#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <visualization_msgs/Marker.h>

namespace LocomotionPlanning{
    class LocomotionPlanner{
        private:
            ros::NodeHandle nh_;
            ros::Subscriber actionSub_;
            ros::Subscriber towerRectangleSub_;
            ros::Subscriber gameStateSub_;

            ros::Publisher cmdVelPub_;
            ros::Publisher towerPosPpub_;
            ros::Publisher markerPub_;

            VehicleRepresentation::PointVehicle vehicle_;
            activity_monitor::Activity currentGameState_;
            geometry_msgs::Point32 currentPos_;
            float currentRotation_;

            planning2::Action currentAction_;
            planning2::Target currentTarget_;
            std::map<int, std::vector<SteeringBehavior*>> behaviors_;
            std::vector<planning2::Target> targetQueue_;
            std::vector<int> towerIndexes_;  

            std::vector<geometry_msgs::Point32> towers_;
            int currentIndex_;
            int currentTargetIndex_;
            SteeringBehavior* currentBehavior_;
            double player_;

            void actionCallback(planning2::Action aMsg);
            void towerRectangleCallback(const geometry_msgs::PolygonStamped& poly);
            void updateTowerPositions(std::vector<geometry_msgs::Point32> points);
            void gameStateCallback(activity_monitor::Activity activityMessage);
            int matchTowerIndex(geometry_msgs::Point32 point);
            void publishTowerPositions();
            void publishTarget(geometry_msgs::Point32 target);
            SteeringBehavior* getBestBehavior(std::vector<SteeringBehavior*> behaviors);

            void initMap();
            void initTowers();
            bool popTarget();

            
        public:
            LocomotionPlanner();

            void loop();

    };
}