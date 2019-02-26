#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Twist.h>


#include <vector>

namespace Navigation{
    class BasicNavigation{
        private:
            ros::NodeHandle nh_;
            ros::Subscriber playerSub_;
            ros::Subscriber velSub_;
            ros::Subscriber towerRectangleSub_;
            ros::ServiceClient goalServiceClient_;

            ros::Publisher velPub_;
            ros::Publisher markerPub_;


            std::string velTopic_;
            std::string playerTopic_;
            std::string towerRectangleTopic_;
            std::string velPubTopic_;

            int num_towers_;
            std::vector<geometry_msgs::Point32> towers_;
            
            int currentTarget_;
            bool imminentCollision_;
            bool decelerating_;


            float maxSpeed_;
            float acceleration_;
            float toleranceDelta_;
            float toleranceSpeed_;
            float collisionDistanceTreshold_;
            float toleranceSpeedCollision_;

            tf::TransformListener listener_;
            geometry_msgs::Vector3 currentVel_;
            geometry_msgs::Point32 current_pos_;
            float currentRotationWrtMap_;
            float playerDirection_;
            float playerDistance_;
            float desiredDirection_;

            geometry_msgs::Twist cmd_;

            // Generate velocity to reach target with linear motion
            geometry_msgs::Vector3 generateDesriredVel();

            // Smooth command velocity to reach the desired one
            geometry_msgs::Vector3 accelerate();
            geometry_msgs::Vector3 decelerate();
            bool monitorCollision();

            // Utility callbacks to get platform information
            void playerCallback(geometry_msgs::PointStamped position);
            void velCallback(geometry_msgs::Twist vel);
            void updateCurrentPos();


            // Utility functions to aim at towers
            void towerRectangleCallback(const geometry_msgs::PolygonStamped& poly);
			void updateTowerPositions(std::vector<geometry_msgs::Point32> points);
			int matchTowerIndex(geometry_msgs::Point32 point);
            void publishTarget(geometry_msgs::Point32 point);

            // Utility functions to perform vector operations
            float magnitude(geometry_msgs::Vector3 vector);

        public:
            BasicNavigation();
            void run();
    };
}

#endif