/* main.cpp -- This file is part of the robogame_kinectfeatures_extractor ROS node created for
 * the purpose of extracting relevant motion features from images.
 *
 * Copyright (C) 2016 Ewerton Lopes
 *
 * LICENSE: This is a free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License (LGPL v3) as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version. This code is distributed in the hope
 * that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU Lesser General Public License (LGPL v3): http://www.gnu.org/licenses/.
 */

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <stdio.h>
#include <iomanip>
#include <time.h>
#include <signal.h>
#include <utility>
#include <cstdlib>
#include <math.h>       /* sqrt */
#include <boost/circular_buffer.hpp>

/* ROS related includes */
#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
/* ... */

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <heartbeat/HeartbeatClient.h>

/* OpenCV related includes */
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
/* ... */

/* Kinect library header includes */
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
/* ... */

/* Local includes */
#include "utils.h"
#include "common.h"
/* ... */

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <kinect_tracker/ContractionIndex.h>
#include <kinect_tracker/PlayerDistance.h>
#include <kinect_tracker/PlayerScreenAngle.h>
#include <kinect_tracker/PlayerPixelPosition.h>

//Create a publisher object.
ros::Publisher pub;
ros::Publisher pub_angle_wrt_screen_center;
ros::Publisher pub_message;
ros::Publisher pub_centres;
ros::Publisher pub_player_pos_wrt_map;
ros::Publisher pub_player_pos_wrt_robot;
ros::Publisher pub_ci;
ros::Publisher pub_player_distance;
ros::Publisher pub_pixel_position;

image_transport::Publisher pub_result_image;

int cam_width;
int cam_height;
float width_fov;
float height_fov;
float seg_threshold;

/* HSV space variables for blob detection */
int hMin = 127;
int sMin = 99;
int vMin = 39;
int hMax = 165;
int sMax = 256;
int vMax = 256;
/* ... */

cv::Point2f blob_center;    // variable for blob center tracking at time t.
cv::Point2f pr_blob_center; // variable for blob center tracking at time t-1.

float mean_distance = 0;    // distance feature
float pr_mean_distance;     // variable for distance at time t-1.
float ci = 0;               // contraction index
float pr_ci = 0;            // variable for ci at time t-1.

bool is_player_missing = false;     // a flag for the player presence.
bool is_exit = false;
bool is_shutdown = false;
bool show_frame = false;

boost::circular_buffer<cv::Point2f> pts(TRAIL_BUFFER_SIZE); // The blob location history
tf::TransformListener* tfListener;  // Global tf listener pointer

cv::Mat seg_color_frame;    // the segmented color frame.
cv::Mat seg_target;
cv::Mat rgbmat;

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;

/* Interruption handler function. In this case, set the variable that controls the
 frame aquisition, breaking the loop, cleaning variables and exit the program elegantly*/
void sigint_handler(int s){
    is_shutdown = true;
}

void callback(const sensor_msgs::ImageConstPtr &depth, const sensor_msgs::ImageConstPtr &image){
    
    cv_bridge::CvImagePtr cv_depth_ptr;
    cv_bridge::CvImagePtr cv_rgb_ptr;
    
    try{
        // Get depth image as matrix
        cv_depth_ptr = cv_bridge::toCvCopy(depth);
        cv_rgb_ptr = cv_bridge::toCvCopy(image);
    }catch (cv_bridge::Exception& e){
        ROS_ERROR("%s",e.what());
        return;
    }

    cv::Mat depmat = cv_depth_ptr->image;
    cv::Mat rgbmat = cv_rgb_ptr->image;
    
    int found_blob = findColorBlob(rgbmat, blob_center);
    
    cv::Mat segmat = Mat::zeros(depmat.size(), depmat.type());
    segmat.convertTo(segmat,CV_32FC1,  0.001);

    /* THIS LOOP COMPUTES A REGION OF INTEREST (A CIRCLE) BASED ON THE mainCenter VARIABLE COMPUTED BY
    THE findColorBlob METHOD. THE IDEA IS THEN TO ASSESS THE MEAN DISTANCE (PIXEL VALUES)
    DEFINED IN THIS AREA AND THUS OBTAIN THE DISTANCE FEATURE. THE SAME LOOP ALSO CALL THE
    segmentDepth METHOD IN ORDER TO OBTAIN THE CONTRACTION INDEX FEATURE.*/

    int offset = 5; // an offset w.r.t the blob_center. We use it to calc an area around the blob_center.

    if (found_blob && blob_center.x >= 5*offset && blob_center.y >= 5*offset 
        && blob_center.x < depmat.size().width - 5*offset 
        && blob_center.y < depmat.size().height - 5*offset){

        //get the Rect containing the circle:
        cv::Rect area_around_blob_center(blob_center.x-offset,blob_center.y-offset, offset*2, offset*2);
        cv::Mat roi, roiArea;
        try{
            // obtain the image ROI:
            depmat.convertTo(depmat,CV_32F,  1.0);
            roi = cv::Mat(depmat, area_around_blob_center);

            // make a black mask, same size:
            cv::Mat maskROI(roi.size(), roi.type(), cv::Scalar::all(0));

            // with a white, filled circle in it:
            cv::circle(maskROI, cv::Point(offset,offset), offset, cv::Scalar::all(255), -1);

            // combine roi & mask:
            roiArea = roi & maskROI;
        }catch (cv::Exception ex){
            ROS_ERROR("%s",ex.what());
            return;
        }

        ROS_INFO("blob_center.x: %f \t\t blob_center.y: %f", blob_center.x, blob_center.y);
    
        cv:Scalar distance = cv::mean(roi);        // compute mean value of the region of interest.
                                                   // RECALL: the pixels correspond to distance in mm.

        mean_distance = distance[0] / 1000.0f;     // compute distance (in meters)

        // perform segmentation in order to get the contraction index featue.
        ci = segmentDepth(depmat, segmat, blob_center.x, blob_center.y, seg_threshold);
        kinect_tracker::ContractionIndex new_ci;
        new_ci.header.stamp = ros::Time::now();
        new_ci.ci = ci;
        pub_ci.publish(new_ci); // publishes the new ci.
        
        
        // phi is the angular coordinate for the width
        float rho = mean_distance;
        float phi = (0.5 - blob_center.x / cam_width) * width_fov;
        float theta = M_PI / 2 - (0.5 - blob_center.y / cam_height) * height_fov;
        
        float x = rho * sin(theta) * cos(phi);
        float y = rho * sin(theta) * sin(phi);
        float z = rho * cos(theta);

        kinect_tracker::PlayerPixelPosition pixel_position;
        pixel_position.header.stamp = ros::Time::now();
        pixel_position.x = blob_center.x;
        pixel_position.y = blob_center.y;
        pub_pixel_position.publish(pixel_position); // publishes the blob_center position.
        
        
        kinect_tracker::PlayerScreenAngle angle_error;
        angle_error.header.stamp = ros::Time::now(); 
        angle_error.angle = phi;
        pub_angle_wrt_screen_center.publish(angle_error); // publishes angle

        kinect_tracker::PlayerDistance player_distance;
        player_distance.header.stamp = ros::Time::now();
        player_distance.distance = mean_distance;
        pub_player_distance.publish(player_distance); // publishes distance to player
        
        ROS_DEBUG("rho:\t%.2f\tphi:\t%.2f°\ttheta:\t%.2f°", rho, phi*180/M_PI, theta*180/M_PI);
        ROS_DEBUG("x:\t%.2f\ty:\t%.2f\tz:\t%.2f", x, y, z);
        
        // TF-Broadcaster
        static tf::TransformBroadcaster br;
        tf::StampedTransform playerTransform;
        
        tf::Transform framePlayerTransform;
        framePlayerTransform.setOrigin( tf::Vector3(x, y, z) );
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        framePlayerTransform.setRotation(q);
        ros::Time now = ros::Time::now();
        br.sendTransform(tf::StampedTransform(framePlayerTransform, now, "/kinect2_link", "/player_link"));
        
        try{
            tfListener->waitForTransform("/kinect2_link", ros::Time(0), "/player_link", now, "/map", ros::Duration(1.0));
            tfListener->lookupTransform("/map", "/player_link", now, playerTransform);

            geometry_msgs::PoseStamped globalPlayerPoseMsg;
            globalPlayerPoseMsg.header.stamp = now;
            globalPlayerPoseMsg.header.frame_id = "/map";
            globalPlayerPoseMsg.pose.position.x = playerTransform.getOrigin().x();
            globalPlayerPoseMsg.pose.position.y = playerTransform.getOrigin().y();
            globalPlayerPoseMsg.pose.position.z = playerTransform.getOrigin().z();
            pub_player_pos_wrt_map.publish(globalPlayerPoseMsg);
            
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
        }
            
        geometry_msgs::PoseStamped localPlayerPoseMsg;
        localPlayerPoseMsg.header.stamp = now;
        localPlayerPoseMsg.header.frame_id = "/kinect2_link";
        localPlayerPoseMsg.pose.position.x = framePlayerTransform.getOrigin().x();
        localPlayerPoseMsg.pose.position.y = framePlayerTransform.getOrigin().y();
        localPlayerPoseMsg.pose.position.z = framePlayerTransform.getOrigin().z();
        pub_player_pos_wrt_robot.publish(localPlayerPoseMsg);
    }

	if (show_frame){
		cv::imshow("view", rgbmat);
		cv::imshow("seg", segmat);
		int key = cv::waitKey(30);
	}
}

// Connection callback that unsubscribes from the tracker if no one is subscribed.
void connectCallback(image_transport::SubscriberFilter &sub_col,
                     image_transport::SubscriberFilter &sub_dep,
                     image_transport::ImageTransport &it) {
                     
    ROS_DEBUG("Player tracker: New subscribers. Subscribing.");
    sub_col.subscribe(it,sub_col.getTopic().c_str(),1);
    sub_dep.subscribe(it,sub_dep.getTopic().c_str(),1);

}

// Replacement SIGINT handler
void onShutdown(int sig){
    ROS_INFO_STREAM("Exiting...");
    is_exit = true;
}

int main(int argc, char** argv){
    
    ros::init(argc, argv, "kinect_tracker");
    ros::NodeHandle nh;
    ros::Rate rate(100);
    
	pub_player_pos_wrt_map = nh.advertise<geometry_msgs::PoseStamped> ("kinect2/player_global_position",1000);
  	pub_player_pos_wrt_robot  = nh.advertise<geometry_msgs::PoseStamped> ("kinect2/player_local_position",1000);
    pub_angle_wrt_screen_center = nh.advertise<kinect_tracker::PlayerScreenAngle>("kinect2/player_relative_angle",1000);
    pub_ci = nh.advertise<kinect_tracker::ContractionIndex>("kinect2/player_contraction_index",1000);
    pub_player_distance = nh.advertise<kinect_tracker::PlayerDistance>("kinect2/player_distance",1000);
    pub_pixel_position = nh.advertise<kinect_tracker::PlayerPixelPosition>("kinect2/player_pixel_position",1000);
  	 
  	tfListener = new tf::TransformListener();
    
    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, onShutdown);
    
    // HeartbeatClient Initialize.
    HeartbeatClient hb(nh, 0.5);
	hb.start();

    heartbeat::State::_value_type state = heartbeat::State::INIT;
    hb.setState(state);
    
    // Declare variables that can be modified by launch file or command line.
    int queue_size;
    std::string cam_ns;
    std::string color_frame_name;
    std::string depth_frame_name;

    std::string pub_topic_centres;
    std::string pub_topic_ubd;
    std::string pub_topic_result_image;
    std::string pub_topic_detected_persons;
    std::string topic_color_image;
    std::string topic_depth_image;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("queue_size", queue_size, int(5));
    private_node_handle_.param("camera_namespace", cam_ns, string("/kinect2"));
    private_node_handle_.param("show_frame", show_frame, show_frame);
    private_node_handle_.param("color_frame", color_frame_name, string("/color_image"));
    private_node_handle_.param("depth_frame", depth_frame_name, string("/depth_image"));
    private_node_handle_.param("cam_width", cam_width, cam_width);
    private_node_handle_.param("cam_height", cam_height, cam_height);
    private_node_handle_.param("width_fov", width_fov, width_fov);
    private_node_handle_.param("height_fov", height_fov, height_fov);
    private_node_handle_.param("seg_threshold", seg_threshold, seg_threshold);
	
    topic_color_image = cam_ns + color_frame_name;
    topic_depth_image = cam_ns + depth_frame_name;
    
    ROS_INFO_STREAM("Color image topic: " << topic_color_image);
    ROS_INFO_STREAM("Color depth topic: " << topic_depth_image);
	ROS_INFO_STREAM("Show frames: " << (show_frame ? "True" : "False"));
	
	if (show_frame){
		cv::namedWindow("view", 1);
		cv::namedWindow("seg", 1);
		cv::startWindowThread();
	}
	
	// Printing queue size
    ROS_DEBUG("Player Tracker: Queue size for synchronisation is set to: %i", queue_size);

    // Image transport handle
    image_transport::ImageTransport it(private_node_handle_);

    // Create a subscriber.
    // Set queue size to 1 because generating a queue here will only pile up images and delay the output by the amount of queued images
    image_transport::SubscriberFilter subscriber_depth;
    image_transport::SubscriberFilter subscriber_image;

    subscriber_depth.subscribe(it, topic_depth_image.c_str(),1);
    subscriber_image.subscribe(it, topic_color_image.c_str(),1);
	
    //The real queue size for synchronisation is set here.
    sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy(queue_size);
    MySyncPolicy.setAgePenalty(1000); //set high age penalty to publish older data faster even if it might not be correctly synchronized.


    // Create synchronization policy. Here: async because time stamps will never match exactly
    const sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> MyConstSyncPolicy = MySyncPolicy;
    Synchronizer< sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> > sync(MyConstSyncPolicy,
                                                                                       subscriber_depth,
                                                                                       subscriber_image);
    // Register one callback for all topics
    sync.registerCallback(boost::bind(&callback, _1, _2));
    
    // set heartbeat node state to started
    state = heartbeat::State::STARTED;
    bool success = hb.setState(state);

    while(ros::ok() && !is_exit){
        // Issue heartbeat.
        hb.alive();
	    ros::spinOnce();
        // Wait until it's time for another iteration.
        rate.sleep() ;
    }
    success = hb.setState(heartbeat::State::STOPPED);
    // Issue heartbeat.
    hb.alive();
    hb.stop();
    return 0;
}
