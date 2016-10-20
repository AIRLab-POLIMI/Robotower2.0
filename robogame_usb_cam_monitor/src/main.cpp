// This program subscribes to wiimote and publishes the interested button state.
#include<ros/ros.h>
#include<sensor_msgs/CompressedImage.h>
#include<robogame_usb_cam_monitor/Info.h>
#include <boost/circular_buffer.hpp>
#include <ctime>
#include <cmath>

const int BUFFER = 10;          // the object trail size
boost::circular_buffer<float> stamp_diff(BUFFER);    // The blob location history

// Create a publisher object for button state.
ros::Publisher pub;
ros::Time previous;

float calculateAverage(){
    float sum = 0;
    for (int i=1; i < (stamp_diff.size()-1); i++){
        sum += stamp_diff[i];
	}
    return sum / stamp_diff.size();
}

float standard_deviation(){
    float mean=0.0, sum_deviation=0.0;
    for (int i=1; i < (stamp_diff.size()-1); i++){
        mean += stamp_diff[i];
    }
    mean /= stamp_diff.size();
    for (int i=1; i < (stamp_diff.size()-1); i++){
        sum_deviation += (stamp_diff[i]-mean)*(stamp_diff[i]-mean);
    }
    return sqrt(sum_deviation/(stamp_diff.size()-1));
}

// A callback function . Executed each time a wiimote message arrives
void MessageReceived(const sensor_msgs::CompressedImage& msg){
    robogame_usb_cam_monitor::Info newMsg;
    if (stamp_diff.size() == 0){
        previous = msg.header.stamp;
        stamp_diff.push_front(0);
        std::cout << "0,";
    }else{
        ros::Duration diff = msg.header.stamp - previous;
        stamp_diff.push_front(diff.toSec());
        previous = msg.header.stamp;
        newMsg.deviation = diff.toSec();
        std::cout << diff << ",";
    }
    newMsg.header.stamp = ros::Time::now();
    newMsg.avg = calculateAverage();
    std::cout << newMsg.avg << ",";
    float stdev = standard_deviation();
    std::cout << stdev << std::endl;
    pub.publish(newMsg);
}

int main (int argc, char** argv){
	// Initialize the ROS system and become a node .
	ros::init(argc, argv, "usb_cam_monitor");
	ros::NodeHandle nh;

	// Create a subscriber object.
	ros::Subscriber sub = nh.subscribe("ext_usb_camera/image/compressed", 1000, &MessageReceived);
	pub = nh.advertise<robogame_usb_cam_monitor::Info>("usb_cam_monitor/info",1000);				// advertise LED and Buzzer state

    //ROS_INFO_STREAM("Listening to incoming external usb camera data...");
    //ROS_INFO_STREAM("Topic: ext_usb_camera/image/compressed");

	// Let ROS take over .
	ros::spin();
}
