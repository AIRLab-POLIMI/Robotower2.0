#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>


namespace player_tracking{

    class PlayerPositionEstimator{
        private:
            ros::NodeHandle nh_;
            ros::Subscriber playerCloudSub_;

            ros::Publisher playerPosPub_;

            geometry_msgs::Point32 calculateCentroid(sensor_msgs::PointCloud playerCloud);
            void playerCloudCallback(sensor_msgs::PointCloud playerCloud);
        public:

            PlayerPositionEstimator();

    };
}