#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <aruco_msgs/MarkerArray.h>
#include <aruco_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/CameraInfo.h>
#include <vector>
#include <string>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#define WIDTH_FOV 70.6*M_PI/180
#define HEIGHT_FOV 60.0*M_PI/180

#define SCREEN_HEIGHT 1080
#define SCREEN_WIDTH 1920


ros::Publisher pose_sup;


void callback(const aruco_msgs::MarkerArray::ConstPtr &msg){

    for (int i= 0; i < msg->markers.size(); i++){
        geometry_msgs::PoseStamped pose;

        std::string tf_name("marker_");
        tf_name += std::to_string(msg->markers[i].id);

        pose.header.stamp = msg->markers[i].header.stamp;
        pose.header.frame_id = tf_name.c_str();

        // phi is the angular coordinate for the width
        float rho = msg->markers[i].pose.pose.position.z;
        float phi = (0.5 - msg->markers[i].pixel_position.x / SCREEN_WIDTH) * WIDTH_FOV;
        float theta = M_PI / 2 - (0.5 - msg->markers[i].pixel_position.y / SCREEN_HEIGHT) * HEIGHT_FOV;
        
        float x = rho * sin(theta) * cos(phi);
        float y = rho * sin(theta) * sin(phi);
        float z = rho * cos(theta);

        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;

        pose.pose.orientation.x = msg->markers[i].pose.pose.orientation.x; 
        pose.pose.orientation.y = msg->markers[i].pose.pose.orientation.y;
        pose.pose.orientation.z = msg->markers[i].pose.pose.orientation.z;
        pose.pose.orientation.w = msg->markers[i].pose.pose.orientation.w;

        //ROS_INFO_STREAM("\tx:" << x <<"\ty: " << y << "\tz:" << z);
        pose_sup.publish(pose);

        // TF-Broadcaster
        static tf::TransformBroadcaster br;
        tf::StampedTransform playerTransform;
        
        tf::Transform poseTransform;
        poseTransform.setOrigin( tf::Vector3(x, y, z) );
        tf::Quaternion q(msg->markers[i].pose.pose.orientation.x,
                         msg->markers[i].pose.pose.orientation.y,
                         msg->markers[i].pose.pose.orientation.z,
                         msg->markers[i].pose.pose.orientation.w);
        
        poseTransform.setRotation(q);
        br.sendTransform(tf::StampedTransform(poseTransform, msg->markers[i].header.stamp, "/kinect2_link",
                                                tf_name.c_str()));


    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "aruco_camera_to_map");
    ros::NodeHandle node;
    ROS_INFO_STREAM("Starting node...");
    pose_sup = node.advertise<geometry_msgs::PoseStamped>("game/marker", 10);
    ros::Subscriber sub = node.subscribe("/aruco_marker_publisher/markers",10,&callback);
    ROS_INFO_STREAM("Listening to aruco marker node...");
    ROS_WARN("Markers orientations are not with respect to camera...");
    ros::spin();
    return 0;
};

