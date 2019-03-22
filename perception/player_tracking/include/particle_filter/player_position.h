#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <player_tracker/LegArray.h>
#include <player_tracker/Leg.h>


namespace player_tracking{

    class PlayerPositionEstimator{
        private:
            ros::NodeHandle nh_;
            ros::Subscriber playerCloudSub_;
            ros::Subscriber playerLegSub_;
            

            ros::Publisher playerPosPub_;
            ros::Publisher markerPub_;

            geometry_msgs::Point32 lastKnownPosition_;

            geometry_msgs::Point32 calculateCentroid(sensor_msgs::PointCloud playerCloud);
            void playerCloudCallback(sensor_msgs::PointCloud playerCloud);
            void playerLegCallback(player_tracker::LegArray playerLegs);
            double calculateLegDistance(player_tracker::Leg leg1, player_tracker::Leg leg2);
            void visualizePLayerLegArray(std::vector<player_tracker::Leg> playerLegs);
        public:

            PlayerPositionEstimator();

    };
}