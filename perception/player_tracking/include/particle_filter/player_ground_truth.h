#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <player_tracker/LegArray.h>
#include <player_tracker/Leg.h>


namespace player_tracking{

    class PlayerPositionRepublisher{
        private:
            ros::NodeHandle nh_;
            ros::Subscriber optitrackSubscriber_;
            ros::Publisher playerPositionPublisher_; 
            geometry_msgs::Pose2D position2D_;

            void optitrackCallback(geometry_msgs::Pose2D position2D);

        public:

            PlayerPositionRepublisher();
            void publishPosition();

    };
}