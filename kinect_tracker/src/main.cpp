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
#include <ground_plane_estimation/GroundPlane.h>
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

#define WIDTH_FOV 70.6*M_PI/180;
#define HEIGHT_FOV 60.0*M_PI/180;

//Create a publisher object.
ros::Publisher pub;
ros::Publisher position_pub;
ros::Publisher pub_message;
ros::Publisher pub_centres;
ros::Publisher player_global_pose_pub;
ros::Publisher player_local_pose_pub;
image_transport::Publisher pub_result_image;
std::string topic_color_image;

// TODO: Refactor this node.

/* HSV space variables for blob detection */
int hMin = 127;
int sMin = 99;
int vMin = 39;
int hMax = 165;
int sMax = 256;
int vMax = 256;
/* ... */

cv::Point2f blobCenter;         // variable for blob center tracking at time t.
cv::Point2f previousBlobCenter;
// FEATURE VARIABLES
float meanDistance = 0;         // distance feature
float ci = 0;                   // contraction index
// previous feature values
float previousDistance;         // variable for distance at time t-1.
float previousCI;               // variable for ci at time t-1.


boost::circular_buffer<cv::Point2f> pts(TRAIL_BUFFER_SIZE); // The blob location history

// ..
bool isPlayerMissing;              // a flag for the player presence.
bool isExit(false);

cv::Mat segmentedColorFrame;    // the segmented color frame.
cv::Mat segmentedTarget;
cv::Mat rgbmat;

using namespace std;
using namespace ground_plane_estimation;
using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;


// Global tf listener pointer
tf::TransformListener* tfListener;


bool is_shutdown = false;
bool show_frame(false);

/* Interruption handler function. In this case, set the variable that controls the
 frame aquisition, breaking the loop, cleaning variables and exit the program elegantly*/
void sigint_handler(int s){
    is_shutdown = true;
}

int asd_counter = 0;

string asdasd(float f){
    stringstream s;
    int N = 5;
    int n = (int)(f);

    if(n>=N) n = N;

    for(int i = 0; i <= N; i++) if (i<=n) s << "#" ; else s << " ";

    return s.str();

}

// cv::Scalar median (cv::Mat image)
// {
//     double m=(image.rows*image.cols)/2;
//     int bin0=0, bin1=0, bin2=0;
//     cv::Scalar med;
//     med.val[0]=-1;
//     med.val[1]=-1;
//     med.val[2]=-1;
//     int histSize = 256;
//   float range[] = { 0, 256 } ;
//   const float* histRange = { range };
//   bool uniform = true;
//   bool accumulate = false;
//   cv::Mat hist0, hist1, hist2;
//     std::vector<cv::Mat> channels;
//   cv::split( image, channels );
//   cv::calcHist( &channels[0], 1, 0, cv::Mat(), hist0, 1, &histSize, &histRange, uniform, accumulate );
//     // cv::calcHist( &channels[1], 1, 0, cv::Mat(), hist1, 1, &histSize, &histRange, uniform, accumulate );
//     // cv::calcHist( &channels[2], 1, 0, cv::Mat(), hist2, 1, &histSize, &histRange, uniform, accumulate );

//     for (int i=0; i<256 && ( med.val[0]<0 || med.val[1]<0 || med.val[2]<0);i++)
//     {
//         bin0=bin0+cvRound(hist0.at<float>(i));
//         // bin1=bin1+cvRound(hist1.at<float>(i));
//         // bin2=bin2+cvRound(hist2.at<float>(i));
//         if (bin0>m && med.val[0]<0)
//             med.val[0]=i;
//         // if (bin1>m && med.val[1]<0)
//         //     med.val[1]=i;
//         // if (bin2>m && med.val[2]<0)
//         //     med.val[2]=i;
//     }

//     return med;
// }




void callback(const sensor_msgs::ImageConstPtr &depth, const sensor_msgs::ImageConstPtr &image, const ground_plane_estimation::GroundPlane::ConstPtr &gp, const sensor_msgs::CameraInfoConstPtr &info)
{
  try
  {
     
    // Get depth image as matrix
    cv_bridge::CvImagePtr cv_depth_ptr = cv_bridge::toCvCopy(depth);
    cv_bridge::CvImagePtr cv_rgb_ptr = cv_bridge::toCvCopy(image);

    cv::Mat depmat = cv_depth_ptr->image;
    cv::Mat rgbmat = cv_rgb_ptr->image;
    
    trackUser(rgbmat);
    
    cv::Mat segmat = Mat::zeros(depmat.size(), depmat.type());
    segmat.convertTo(segmat,CV_32FC1,  0.001);

    /* THIS LOOP COMPUTES A REGION OF INTEREST (A CIRCLE) BASED ON THE mainCenter VARIABLE COMPUTED BY
	   THE trackUser METHOD. THE IDEA IS THEN TO ASSESS THE MEAN DISTANCE (PIXEL VALUES)
	   DEFINED IN THIS AREA AND THUS OBTAIN THE DISTANCE FEATURE. THE SAME LOOP ALSO CALL THE
	   segmentDepth METHOD IN ORDER TO OBTAIN THE CONTRACTION INDEX FEATURE.*/
    
    int radius = 5;
   
    if ((blobCenter.x != -1000) && (blobCenter.x != 0) && blobCenter.x >= 5*radius && blobCenter.y >= 5*radius && blobCenter.x < depmat.size().width - 5*radius && blobCenter.y < depmat.size().height - 5*radius ){
    // if ((blobCenter.x != -1000) && (blobCenter.x != 0)){

        //get the Rect containing the circle:
        cv::Rect r(blobCenter.x-radius, blobCenter.y-radius, radius*2, radius*2);
        cv::Mat roi, roiArea;
        try{
            // obtain the image ROI:
            depmat.convertTo(depmat,CV_32F,  1.0);
            roi = cv::Mat(depmat, r);

            // make a black mask, same size:
            cv::Mat maskROI(roi.size(), roi.type(), cv::Scalar::all(0));

            // with a white, filled circle in it:
            cv::circle(maskROI, cv::Point(radius,radius), radius, cv::Scalar::all(255), -1);

            // combine roi & mask:
            roiArea = roi & maskROI;
            // -------
        } catch (cv::Exception ex) {
			ROS_ERROR("%s",ex.what());
            return;
		}

       ROS_INFO("blobCenter.x: %f \t\t blobCenter.y: %f", blobCenter.x, blobCenter.y);
        // ROS_INFO("###################### roi.cols: %i \t\t roi.rows: %i", roi.cols, roi.rows);
        // if(roi.cols < 10 || roi.rows < 10) ROS_INFO("###################### roi.cols < 10 || roi.rows < 10");

        cv:Scalar distance = cv::mean(roi);        // compute mean value of the region of interest.
                                                    //    RECALL: the pixels correspond to distance in mm.


        cout << std::fixed << std::setprecision( 3 );
        cout << endl;
        for (int i = 0; i < roi.cols; i++){
            for(int j = 0; j < roi.rows; j++){
                cout << roi.at<float>(i, j)/1000.0 << " ";
            }
            cout << endl;
        }

        cout << endl;
        for (int i = 0; i < roi.cols; i++){
            for(int j = 0; j < roi.rows; j++){
                cout << asdasd(roi.at<float>(i, j)/1000.0);
            }
            cout << endl;
        }

        cout << asd_counter++ << endl;


        // cv::Scalar med; //0:1st channel, 1:2nd channel and 2:3rd channel
        // med = median(roi);
        // std::cout<<"Median: "<<med.val[0]<<std::endl;
        meanDistance = distance[0] / 1000.0f;  // compute distance (in meters)
        // float medianDistance = cv::median(roi) / 1000.0f;
        cv::Scalar meanDistance_scalar;
        cv::Scalar stdDevDistance_scalar;
        cv::meanStdDev(roi, meanDistance_scalar, stdDevDistance_scalar);
       
        float stdDevDistance = stdDevDistance_scalar[0];

        cout << "mean: " << meanDistance
            // << "\tmedian: " << med.val[0]
            << "\tstdDev: " << stdDevDistance << endl;

        if(stdDevDistance < 200){
                

            // perform segmentation in order to get the contraction index featue.
            // The result will be saved in ci variable//
            //test_other();
            segmentDepth(depmat, segmat, blobCenter.x, blobCenter.y, ci, 300);
            
            
            // phi is the angular coordinate for the width
            float rho = meanDistance;
            float phi = (0.5 - blobCenter.x / info->width) * WIDTH_FOV;
            float theta = M_PI / 2 - (0.5 - blobCenter.y / info->height) * HEIGHT_FOV;
            
            float x = rho * sin(theta) * cos(phi);
            float y = rho * sin(theta) * sin(phi);
            float z = rho * cos(theta);
            
            std_msgs::Float64 angle_error;
			angle_error.data = phi;

			position_pub.publish(angle_error); // publishes angle
            
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
                player_global_pose_pub.publish(globalPlayerPoseMsg);
                
            } catch (tf::TransformException ex) {
                ROS_ERROR("%s",ex.what());
            }
                
            geometry_msgs::PoseStamped localPlayerPoseMsg;
            localPlayerPoseMsg.header.stamp = now;
            localPlayerPoseMsg.header.frame_id = "/kinect2_link";
            localPlayerPoseMsg.pose.position.x = framePlayerTransform.getOrigin().x();
            localPlayerPoseMsg.pose.position.y = framePlayerTransform.getOrigin().y();
            localPlayerPoseMsg.pose.position.z = framePlayerTransform.getOrigin().z();
            player_local_pose_pub.publish(localPlayerPoseMsg);
        } else {
            cout << "##### high std dev !!! " << endl;
        }
    }
   
   
	if (show_frame){
		cv::imshow("view", rgbmat);
		cv::imshow("seg", segmat);
		int key = cv::waitKey(30);
	}
  }
  
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", depth->encoding.c_str());
  }
}

// Connection callback that unsubscribes from the tracker if no one is subscribed.
void connectCallback(message_filters::Subscriber<CameraInfo> &sub_cam,
                     message_filters::Subscriber<GroundPlane> &sub_gp,
                     image_transport::SubscriberFilter &sub_col,
                     image_transport::SubscriberFilter &sub_dep,
                     image_transport::ImageTransport &it) {
                     
    ROS_DEBUG("Upper Body Detector: New subscribers. Subscribing.");
    sub_cam.subscribe();
    sub_gp.subscribe();
    sub_col.subscribe(it,sub_col.getTopic().c_str(),1);
    sub_dep.subscribe(it,sub_dep.getTopic().c_str(),1);

}

// Replacement SIGINT handler
void onShutdown(int sig){
    ROS_INFO_STREAM("Exiting...");
    isExit = true;
}

int main(int argc, char** argv)
{
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "kinect_tracker");
    ros::NodeHandle nh;
	// Loop at 100Hz until the node is shutdown.
    ros::Rate rate(100);
    
	player_global_pose_pub = nh.advertise<geometry_msgs::PoseStamped> ("robogame/player_global_position",1000);
  	player_local_pose_pub  = nh.advertise<geometry_msgs::PoseStamped> ("robogame/player_local_position",1000);
    position_pub = nh.advertise<std_msgs::Float64>("robogame/player_relative_angle",1000);
  	 
  	tfListener = new tf::TransformListener();
 	
 	// Create the blob detection image panel together with the
	// sliders for run time adjustments.
	/*
	cv::namedWindow("mask", 1);

	cv::createTrackbar("hMin", "mask", &hMin, 256);
	cv::createTrackbar("sMin", "mask", &sMin, 256);
	cv::createTrackbar("vMin", "mask", &vMin, 256);
	cv::createTrackbar("hMax", "mask", &hMax, 256);
	cv::createTrackbar("sMax", "mask", &sMax, 256);
	cv::createTrackbar("vMax", "mask", &vMax, 256);*/
    
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
    string cam_ns;
    string topic_gp;

    string pub_topic_centres;
    string pub_topic_ubd;
    string pub_topic_result_image;
    string pub_topic_detected_persons;


    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("queue_size", queue_size, int(5));

    private_node_handle_.param("camera_namespace", cam_ns, string("/kinect2"));
    private_node_handle_.param("ground_plane", topic_gp, string("/ground_plane"));
    private_node_handle_.param("show_frame", show_frame, show_frame);
	
	
    topic_color_image = cam_ns + "/sd/image_color_rect";
    ROS_INFO_STREAM("COLOR IMAGE TOPIC: " << topic_color_image);
    string topic_depth_image = cam_ns + "/sd/image_depth_rect";
    ROS_INFO_STREAM("COLOR DEPTH TOPIC: " << topic_depth_image);
    string topic_camera_info = cam_ns + "/sd/camera_info";
	ROS_INFO_STREAM("COLOR IMAGE INFO TOPIC: " << topic_camera_info);
	ROS_INFO_STREAM("SHOW FRAMES: " << (show_frame ? "True" : "False"));
	
	if (show_frame){
		ROS_INFO("Initiating frames!");
		cv::namedWindow("view", 1);
		cv::namedWindow("seg", 1);
		cv::startWindowThread();
	}
	
	
	// Printing queue size
    ROS_DEBUG("upper_body_detector: Queue size for synchronisation is set to: %i", queue_size);

    // Image transport handle
    image_transport::ImageTransport it(private_node_handle_);

    // Create a subscriber.
    // Set queue size to 1 because generating a queue here will only pile up images and delay the output by the amount of queued images
    image_transport::SubscriberFilter subscriber_depth;
    image_transport::SubscriberFilter subscriber_image;

    subscriber_depth.subscribe(it, topic_depth_image.c_str(),1);
    subscriber_image.subscribe(it, topic_color_image.c_str(),1);
    
    message_filters::Subscriber<CameraInfo> subscriber_camera_info(nh, topic_camera_info.c_str(), 1);
    message_filters::Subscriber<GroundPlane> subscriber_gp(nh, topic_gp.c_str(), 1);


	
    //The real queue size for synchronisation is set here.
    sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, GroundPlane, CameraInfo> MySyncPolicy(queue_size);
    MySyncPolicy.setAgePenalty(1000); //set high age penalty to publish older data faster even if it might not be correctly synchronized.


    // Create synchronization policy. Here: async because time stamps will never match exactly
    const sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image,GroundPlane,CameraInfo> MyConstSyncPolicy = MySyncPolicy;
    Synchronizer< sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, GroundPlane,CameraInfo> > sync(MyConstSyncPolicy,
                                                                                       subscriber_depth,
                                                                                       subscriber_image,
                                                                                       subscriber_gp,
                                                                                       subscriber_camera_info);
    // Register one callback for all topics
    sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));
    
    // set heartbeat node state to started
    state = heartbeat::State::STARTED;
    bool success = hb.setState(state);

    while(ros::ok() && !isExit){
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
