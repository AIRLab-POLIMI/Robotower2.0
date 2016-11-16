#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "/home/airlab/catkin_ws/devel/include/r2p/Velocity.h"
#include "geometry_msgs/Twist.h"

#define LINEAR 2;
#define ANGULAR 5;

class SubscribeAndPublish{
	public:
		SubscribeAndPublish(){
			//Topic su cui voglio pubblicare qualcosa
			pub = n.advertise<r2p::Velocity>("triskar/velocity", 1000);
			//Topic da cui voglio leggere
			sub = n.subscribe("spacenav/twist", 1000, &SubscribeAndPublish::callback, this);
		}	

		void callback(const geometry_msgs::Twist& message){
			r2p::Velocity output;
			//output.header.stamp = ros::Time::now();
			output.x = (message.linear.x)*LINEAR;
			output.y = -(message.linear.y)*LINEAR;
			output.w = (message.angular.z)*ANGULAR;
			pub.publish(output);
		}
	private:
		ros::NodeHandle n;
		ros::Publisher pub;
		ros::Subscriber sub;
};

int main(int argc, char **argv){
	ros::init(argc, argv, "forwarder");
	SubscribeAndPublish forwarderObject;
	ros::spin();
	return 0;
}
