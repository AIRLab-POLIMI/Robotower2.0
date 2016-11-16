#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "r2p/Velocity.h"
#include "geometry_msgs/Twist.h"

#define LINEAR 4
#define ANGULAR 8
#define PUBLISH_REPEAT 5

int lastmsg = 0;
r2p::Velocity output;

void callback(const geometry_msgs::Twist& message){
        lastmsg = 0;

        output.x = (message.linear.x)*LINEAR;
        output.y = -(message.linear.y)*LINEAR;
        output.w = (message.angular.z)*ANGULAR;
}

int main(int argc, char **argv){
        ros::init(argc, argv, "forwarder_v2");

        ros::NodeHandle n;
        ros::Publisher pub = n.advertise<r2p::Velocity>("triskar/velocity", 5);
        ros::Subscriber sub = n.subscribe("spacenav/twist", 5, callback);
        ros::Rate rate(20);

        while (ros::ok()) {
                ros::spinOnce();
                if (lastmsg++ < PUBLISH_REPEAT) {
                        pub.publish(output);    
                }
                rate.sleep();
        }

        return 0;
}

