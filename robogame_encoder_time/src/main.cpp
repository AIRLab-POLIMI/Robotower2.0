// This program subscribes to wiimote and publishes the interested button state.
#include<ros/ros.h>
#include<ros/console.h>
#include <time.h>
#include<r2p/EncoderStamped.h>

// Create a publisher object for button state.
ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher pub3;

void publishMsg(ros::Publisher& pub, const r2p::EncoderStamped& msg){
    r2p::EncoderStamped newmsg;
    newmsg.header.stamp = ros::Time::now();
    newmsg.encoder.delta = msg.encoder.delta;
    pub.publish(newmsg);
}

// A callback function for wrapping encoder1 msg.
void handle_msg1(const r2p::EncoderStamped& msg){
    ROS_INFO("RECEIVED NEW MESSAGE ENCODER1");
    publishMsg(pub1,msg);
}

// A callback function for wrapping encoder2 msg.
void handle_msg2(const r2p::EncoderStamped& msg){
    ROS_INFO("RECEIVED NEW MESSAGE ENCODER2");
    publishMsg(pub2,msg);
}

// A callback function for wrapping encoder3 msg.
void handle_msg3(const r2p::EncoderStamped& msg){
    ROS_INFO("RECEIVED NEW MESSAGE ENCODER3");
    publishMsg(pub3,msg);
}

int main (int argc, char** argv){
	// Initialize the ROS system and become a node .
	ros::init(argc, argv, "robogame_timestamp");
	ros::NodeHandle nh;

	// Create a subscriber object.
	ros::Subscriber sub1 = nh.subscribe("/triskar/encoder1", 1000, &handle_msg1);
	ros::Subscriber sub2 = nh.subscribe("/triskar/encoder2", 1000, &handle_msg2);
	ros::Subscriber sub3 = nh.subscribe("/triskar/encoder3", 1000, &handle_msg3);
    	pub1 = nh.advertise<r2p::EncoderStamped>("/StampedEncoder1",1000);
	pub2 = nh.advertise<r2p::EncoderStamped>("/StampedEncoder2",1000);
	pub3 = nh.advertise<r2p::EncoderStamped>("/StampedEncoder3",1000);
	// Let ROS take over .
	ros::spin();
}
