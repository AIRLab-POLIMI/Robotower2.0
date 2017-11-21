#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/CameraInfo.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#define WIDTH_FOV 70.6*M_PI/180
#define HEIGHT_FOV 60.0*M_PI/180

ros::Publisher pose_sup;

void callback(const geometry_msgs::PoseStamped::ConstPtr & msg,
              const geometry_msgs::PointStamped::ConstPtr & msg_pixel,
              const sensor_msgs::CameraInfo::ConstPtr & cam_info){
    geometry_msgs::PoseStamped pose;       
    
    ROS_INFO("RECEIVED msg: ");
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "/map";

    // phi is the angular coordinate for the width
    float rho = msg->pose.position.z;
    float phi = (0.5 - msg_pixel->point.x / cam_info->width) * WIDTH_FOV;
    float theta = M_PI / 2 - (0.5 - msg_pixel->point.y / cam_info->height) * HEIGHT_FOV;
    
    float x = rho * sin(theta) * cos(phi);
    float y = rho * sin(theta) * sin(phi);
    float z = rho * cos(theta);

    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;

    pose.pose.orientation.x = msg->pose.orientation.x; 
    pose.pose.orientation.y = msg->pose.orientation.y;
    pose.pose.orientation.z = msg->pose.orientation.z;
    pose.pose.orientation.w = msg->pose.orientation.w;

    ROS_INFO_STREAM("\tx:" << x <<"\ty: " << y << "\tz:" << z);
    pose_sup.publish(pose);

    // TF-Broadcaster
    static tf::TransformBroadcaster br;
    tf::StampedTransform playerTransform;
    
    tf::Transform poseTransform;
    poseTransform.setOrigin( tf::Vector3(x, y, z) );
    tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    //q.setRPY(0, 0, 0);
    poseTransform.setRotation(q);
    ros::Time now = ros::Time::now();
    br.sendTransform(tf::StampedTransform(poseTransform, now, "/kinect2_link", "/pose_aruco_link"));

}

int main(int argc, char** argv){

    ros::init(argc, argv, "aruco_camera_to_map");
    ros::NodeHandle node;

    pose_sup = node.advertise<geometry_msgs::PoseStamped>("/aruco_single/pose_remapped", 10);

    message_filters::Subscriber<geometry_msgs::PoseStamped> aruco_sub(node, "aruco_single/pose", 1);
    message_filters::Subscriber<geometry_msgs::PointStamped> pixel_sub(node,"aruco_single/pixel", 10);
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(node,"/kinect2/qhd/camera_info", 10);


    //The real queue size for synchronisation is set here.
    message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::PointStamped, sensor_msgs::CameraInfo> MySyncPolicy(5);
    MySyncPolicy.setAgePenalty(1000); //set high age penalty to publish older data faster even if it might not be correctly synchronized.


    // Create synchronization policy. Here: async because time stamps will never match exactly
    const message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped,geometry_msgs::PointStamped, sensor_msgs::CameraInfo> MyConstSyncPolicy = MySyncPolicy;
    message_filters::Synchronizer< message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped,
    geometry_msgs::PointStamped, sensor_msgs::CameraInfo> > sync(MyConstSyncPolicy,aruco_sub, pixel_sub, info_sub);

    // Register one callback for all topics
    sync.registerCallback(boost::bind(&callback, _1, _2, _3));

    ros::Rate rate(100);
    
    while (node.ok()){
        rate.sleep();
        ros::spin();
    }

    return 0;
};

