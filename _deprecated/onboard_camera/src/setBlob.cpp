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
#include <math.h>       /* sqrt */

/* ROS related includes */
#include <ros/ros.h>
/* ... */

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

/* OpenCV related includes */
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
/* ... */

cv::Mat rgbmat;

using namespace std;
using namespace cv;

void callback(const sensor_msgs::ImageConstPtr &image){
	try{     
		// Get depth image as matrix
		cv_bridge::CvImagePtr cv_rgb_ptr = cv_bridge::toCvCopy(image);
		rgbmat = cv_rgb_ptr->image;
  	}catch (cv_bridge::Exception& e){
    	ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image->encoding.c_str());
  	}
}

int main(int argc, char** argv)
{
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "onboard_camera_set_blob_node");
    ros::NodeHandle nh;
    
    std::string topic_color_image, color_frame_name, cam_ns;

    /* HSV space variables for blob detection */
    int hMin, sMin, vMin, hMax, sMax, vMax;
    
	// Loop at 100Hz until the node is shutdown.
    ros::Rate rate(100);
	
	nh.getParam("camera_namespace", cam_ns);
    nh.getParam("color_frame", color_frame_name);
    nh.getParam("h_min", hMin);
    nh.getParam("s_min", sMin);
    nh.getParam("v_min", vMin);
    nh.getParam("h_max", hMax);
    nh.getParam("s_max", sMax);
    nh.getParam("v_max", vMax);

    
    topic_color_image = cam_ns + color_frame_name;
 	
 	// Create the blob detection image panel together with the
	// sliders for run time adjustments.
	cv::namedWindow("mask", CV_WINDOW_NORMAL);
  	cv::startWindowThread();
  	image_transport::ImageTransport it(nh);
  	image_transport::Subscriber sub = it.subscribe(topic_color_image.c_str(), 1, callback);

	cv::createTrackbar("hMin", "mask", &hMin, 256);
	cv::createTrackbar("sMin", "mask", &sMin, 256);
	cv::createTrackbar("vMin", "mask", &vMin, 256);
	cv::createTrackbar("hMax", "mask", &hMax, 256);
	cv::createTrackbar("sMax", "mask", &sMax, 256);
	cv::createTrackbar("vMax", "mask", &vMax, 256);

	ROS_INFO_STREAM("Listening to " << topic_color_image << " image.");

    while(ros::ok()){
        
        if (!rgbmat.empty()){
        
        	ROS_DEBUG("Processing image...");
			cv::Mat hsv = cv::Mat::zeros(rgbmat.size(), rgbmat.type()); // define container for the converted Mat.
			cv::cvtColor(rgbmat, hsv, cv::COLOR_BGR2HSV);               // convert
			/* ... */

			/* construct a mask for the color (default to "green"), then perform
			   a series of dilations and erosions to remove any small
			   blobs left in the mask... */
			cv::Mat mask;
			cv::inRange(hsv,cv::Scalar(hMin,sMin,vMin), cv::Scalar(hMax,sMax,vMax), mask);
			cv::Mat erodeElement = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
			cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(8,8));
			// perform erode and dilate operations.
			cv::erode(mask, mask, erodeElement);
			cv::erode(mask, mask, erodeElement);
			cv::dilate(mask, mask, dilateElement);
			cv::dilate(mask, mask, dilateElement);
			cv::imshow("mask",mask);           // exihbit mask.
			int key = cv::waitKey(30);
			/* ... */
		}else{
			ROS_DEBUG("RGB frame is empty");
		}
		
		
		ros::spinOnce();
	    // Wait until it's time for another iteration.
	    rate.sleep();
    }
    return 0;
}
