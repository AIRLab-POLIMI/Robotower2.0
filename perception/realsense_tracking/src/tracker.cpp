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
/* ... */

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <realsense_tracking/PlayerInfo.h>

#include "tracker.h"

#define QUEUE_SIZE 5
#define RATE 30


bool is_exit = false;
bool is_shutdown = false;

void PlayerTracker::kalmanPredict(){
    x = F * x + u;
    P = F*P*F.transpose() + Q;
}

void PlayerTracker::kalmanUpdate(float angle, float distance){
    Eigen::Matrix<float, 1, 2> Z;
    Z << angle, distance;
    auto Y =  Z.transpose() - (H*x);
    auto S = ((H*P) * H.transpose()) + R;
    auto K = (P * H.transpose()) * S.inverse();
    x = x + (K * Y);
    P = (I - (K*H)) * P;
}

Eigen::Matrix<float, 3, 1> PlayerTracker::getSphericalCoordinate(float rho, float theta, float phi){
    float x = rho * sin(theta) * cos(phi);
    float y = rho * sin(theta) * sin(phi);
    float z = rho * cos(theta);
    Eigen::Matrix<float, 3, 1> m;
    m << x,y,z;
    return m;
}

void PlayerTracker::publishKalman(){
    // Defining rho, phi and theta
    float rho = x(1,0);
    float phi = x(0,0);
    float theta = M_PI / 2 - (0.5 - pr_blob_center.y / cam_height) * height_fov;

    // getting spherical coordinate
    Eigen::Matrix<float, 3, 1> pixel_sph_coord = getSphericalCoordinate(rho,theta,phi);    // return column vector (newx,newy,newz) pixel pos.

    ROS_DEBUG_STREAM("Kalman Spherical:\n" << pixel_sph_coord.transpose());
    
    // TF-Broadcaster
    static tf::TransformBroadcaster br;
    tf::Transform framePlayerTransform;
    framePlayerTransform.setOrigin( tf::Vector3(pixel_sph_coord(0,0), pixel_sph_coord(1,0), pixel_sph_coord(2,0)));
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    framePlayerTransform.setRotation(q);
    ros::Time now = ros::Time::now();
    br.sendTransform(tf::StampedTransform(framePlayerTransform, now, "/onboard_link", "/player_filtered_link"));
 /*
    // Publish general FILTERED player data: angle, distance.
    realsense_tracking::PlayerInfo msg;
    msg.header.stamp = ros::Time::now();
    msg.pixel_position.x = -1;  //NOT FILTERED
    msg.pixel_position.y = -1;  //NOT FILTERED
    msg.pixel_position.z = -1;  //NOT FILTERED
    msg.ci = -1;                //NOT FILTERED
    msg.angle = x(0,0);
    msg.distance = x(1,0);
    pub_kalman.publish(msg);*/

}

void PlayerTracker::callback(const sensor_msgs::ImageConstPtr &depth, const sensor_msgs::ImageConstPtr &image){

    ROS_DEBUG("Received");
    
    cv_bridge::CvImagePtr cv_depth_ptr;
    cv_bridge::CvImagePtr cv_rgb_ptr;
    
    try{
        // Get depth image as matrix
        cv_depth_ptr = cv_bridge::toCvCopy(depth);
        cv_rgb_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
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

        ROS_DEBUG("blob_center.x: %f \t\t blob_center.y: %f", blob_center.x, blob_center.y);
    
        cv:Scalar distance = cv::mean(roi);        // compute mean value of the region of interest.
                                                   // RECALL: the pixels correspond to distance in mm.

        mean_distance = distance[0] / 1000.0f;     // compute distance (in meters)

        // perform segmentation in order to get the contraction index featue.
        ci = segmentDepth(depmat, segmat, blob_center.x, blob_center.y, seg_threshold);    
        
        // Defining rho, phi and theta
        float rho = mean_distance;
        float phi = (0.5 - blob_center.x / cam_width) * width_fov;
        float theta = M_PI / 2 - (0.5 - blob_center.y / cam_height) * height_fov;

        // getting spherical coordinate
        Eigen::Matrix<float, 3, 1> pixel_sph_coord = getSphericalCoordinate(rho,theta,phi);    // return column vector (newx,newy,newz) pixel pos.

        // Publish general UNFILTERED player data: CI, pixel_pos, angle, distance.
        // realsense_tracking::PlayerInfo msg;
        // msg.header.stamp = ros::Time::now();
        // msg.pixel_position.x = blob_center.x;
        // msg.pixel_position.y = blob_center.y;
        // msg.pixel_position.z = 0;
        // msg.ci = ci;
        // msg.angle = phi;
        // msg.distance = mean_distance;
        // pub.publish(msg);

        // KALMAN UPDATE
        distance_buffer.push_front(mean_distance);          // push into a simple low-pass filter.
        float mean_of_buffer = accumulate(distance_buffer.begin(), distance_buffer.end(), 0.0)/distance_buffer.size();
        pr_blob_center = blob_center;
        kalmanUpdate(phi, mean_of_buffer);
        ////////////////////

        // ROS_INFO("rho:\t%.2f\tphi:\t%.2f°\ttheta:\t%.2f°", rho, phi*180/M_PI, theta*180/M_PI);
        // ROS_INFO("x:\t%.2f\ty:\t%.2f\tz:\t%.2f", pixel_sph_coord(0,0), pixel_sph_coord(1,0), pixel_sph_coord(2,0));
        
        // TF-Broadcaster
        static tf::TransformBroadcaster br;
        tf::StampedTransform playerTransform;
        
        tf::Transform framePlayerTransform;
        framePlayerTransform.setOrigin( tf::Vector3(pixel_sph_coord(0,0), pixel_sph_coord(1,0), pixel_sph_coord(2,0)) );
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        framePlayerTransform.setRotation(q);
        ros::Time now = ros::Time::now();
        br.sendTransform(tf::StampedTransform(framePlayerTransform, now, "/onboard_link", "/player_link"));
        /*
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
        pub_player_pos_wrt_robot.publish(localPlayerPoseMsg);*/
    }

    if (show_frame){
		cv::imshow("view", rgbmat);
		cv::imshow("seg", depmat);
		int key = cv::waitKey(30);
	}
    
    kalmanPredict(); // Perform Kalman prediction
    publishKalman(); // publish Kalman filtered player tf.

	
}

// Connection callback that unsubscribes from the tracker if no one is subscribed.
void PlayerTracker::connectCallback(image_transport::SubscriberFilter &sub_col,
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

PlayerTracker::PlayerTracker(): distance_buffer(BUFFER_DIST_SIZE){
    mean_distance = 0;    			// distance feature
    pr_mean_distance;     			// variable for distance at time t-1.
    ci = 0;               			// contraction index
    pr_ci = 0;            			// variable for ci at time t-1.
    is_player_missing = false;     // a flag for the player presence.
    is_exit = false;
    is_shutdown = false;
    show_frame = true;

    nh.getParam("camera_namespace", cam_ns);
    nh.getParam("show_frame", show_frame);
    nh.getParam("color_frame", color_frame_name);
    nh.getParam("depth_frame", depth_frame_name);
    nh.getParam("tracker_cam_width", cam_width);
    nh.getParam("tracker_cam_height", cam_height);
    nh.getParam("tracker_width_fov", width_fov);
    nh.getParam("tracker_height_fov", height_fov);
    nh.getParam("seg_threshold", seg_threshold);
    nh.getParam("h_min", hMin);
    nh.getParam("s_min", sMin);
    nh.getParam("v_min", vMin);
    nh.getParam("h_max", hMax);
    nh.getParam("s_max", sMax);
    nh.getParam("v_max", vMax);

	
    topic_color_image = cam_ns + color_frame_name;
    topic_depth_image = cam_ns + depth_frame_name;
    
    ROS_INFO_STREAM("Color image topic: " << topic_color_image);
    ROS_INFO_STREAM("Color depth topic: " << topic_depth_image);
	ROS_INFO_STREAM("Show frames: " << (show_frame ? "True" : "False"));

    // pub_player_pos_wrt_map = nh.advertise<geometry_msgs::PoseStamped> ("kinect2/player_global_position",1);
  	// pub_player_pos_wrt_robot  = nh.advertise<geometry_msgs::PoseStamped> ("kinect2/player_local_position",1);
    // pub = nh.advertise<realsense_tracking::PlayerInfo>("kinect2/player_info",1);
  	// pub_kalman = nh.advertise<realsense_tracking::PlayerInfo>("kinect2/player_filtered_info",1);

  	tfListener = new tf::TransformListener();

    float delta_t = (1./RATE);
    float delta_t2 = pow(delta_t,2)/2;

    // Kalman filter matrices
    x << 0,0,0,0,0,0;
    u << 0,0,0,0,0,0;
    H << 1,0,0,0,0,0,   // measurement function 6 states - 2 observed (angle and distance)
         0,1,0,0,0,0;  

    I << 1,0,0,0,0,0,            // identity matrix
         0,1,0,0,0,0,
         0,0,1,0,0,0,
         0,0,0,1,0,0,
         0,0,0,0,1,0,
         0,0,0,0,0,1;     

    R << 0.1,0,                       // measurement uncertainty (2 uncorrelated measures with uncertainty)
         0,0.3;

    P << 1,0,0,0,0,0,             // initial uncertainty
         0,1,0,0,0,0,
         0,0,1,0,0,0,
         0,0,0,1,0,0,
         0,0,0,0,1,0,
         0,0,0,0,0,1;            
                                          // Transition Matrix
    F << 1,0,delta_t,0,delta_t2,0,        // angle
         0,1,0,delta_t,0,delta_t2,        // polar distance
         0,0,1,0,delta_t,0,               // angular speed
         0,0,0,1,0,delta_t,               // player velocity
         0,0,0,0,1,0,                     // angular acceleration
         0,0,0,0,0,1;                     // player acceleration

    Q << 0.1,0,0,0,0,0,         // process noise matrix
         0,0.1,0,0,0,0,
         0,0,0.01,0,0,0,
         0,0,0,0.01,0,0,
         0,0,0,0,0.001,0,
         0,0,0,0,0,0.001;
}

/* Locate largest color blob on a RGB image. */
int PlayerTracker::findColorBlob(cv::Mat& srcFrame,  cv::Point2f &blob_center){
	/* resize the frame and convert it to the HSV color space... */
	cv::Mat frame(srcFrame.size(), srcFrame.type());               // make copy
	resize(srcFrame,frame);                                        // resize
    cv::Mat hsv = cv::Mat::zeros(frame.size(), frame.type());      // define container for the converted Mat.
	cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);                   // convert

    /* construct a mask for the color, then perform a series of dilations and erosions
	to remove any small blobs left in the mask... */
	cv::Mat mask;
	cv::inRange(hsv,cv::Scalar(hMin,sMin,vMin), cv::Scalar(hMax,sMax,vMax), mask);
	cv::Mat erodeElement = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
	cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(8,8));
    
	// perform erode and dilate operations.
    cv::erode(mask, mask, erodeElement);
	cv::erode(mask, mask, erodeElement);
	cv::dilate(mask, mask, dilateElement);
	cv::dilate(mask, mask, dilateElement);

	//cv::imshow("mask",mask);           // exihbit mask.

	// find contours in the mask and initialize the current blob center
	std::vector<std::vector<cv::Point> > contours;  // container for the contours
	std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask.clone(), contours,
                     hierarchy,                     /* find the image contours */
                     CV_RETR_EXTERNAL,
                     CV_CHAIN_APPROX_SIMPLE);

    cv::Point2f center;                /* container var for the new center. 
													 Set to arbitrary init value*/

	/* Only proceed if at least one contour was found, i.e., if at least one
        object has been tagged as a target (NOTE: that the contour correspond
        to the color blog dettected) */
	if (contours.size() > 0){

		int largest_area=0;               // container for the max area
		int largest_contour_index=0;      // container for the index of the max area found in countours

		/* find the largest contour in the mask, then use
    	   it to compute the minimum enclosing circle and
           centroid.*/
		for(int i=0; i < contours.size();i++){
			// iterate through each contour.
			double contourArea = cv::contourArea(cv::Mat(contours[i]),false);  //  Find the area of contour
            /* if the area is bigger than the lready found one, update it.*/
            if(contourArea > largest_area){
				largest_area=contourArea;
				largest_contour_index=i;
			}
		}

		cv::Point2f temp_center;
  		float blob_radius;
		cv::minEnclosingCircle((cv::Mat)contours[largest_contour_index], temp_center, blob_radius);
		cv::Moments M = cv::moments((cv::Mat)contours[largest_contour_index]);
		center = cv::Point2f(int(M.m10 / M.m00), int(M.m01 / M.m00));

		// Only proceed if the blob_radius is larger them a minimum size.
		if (blob_radius > MIN_BLOB_RADIUS){
			// draw the circle and centroid on the frame,
			// then update the list of tracked points
			cv::circle(frame, cv::Point(int(temp_center.x), int(temp_center.y)), int(blob_radius), cv::Scalar(0, 255, 255), 2);
			cv::circle(frame, center, 5, cv::Scalar(0, 0, 255), -1);
            blob_center = center;            // save the center of the detected circle.
		}

		//pts.push_front(center);		// update the points queue
		blob_center = center;       // update blob center.
		srcFrame = frame.clone();   // update the input frame.
		return 1;
	}
	
	return 0; // Blob not detected
}

void PlayerTracker::resize(cv::Mat& sourceFrame, cv::Mat& resultingFrame, int width, int height){
    // initialize the dimensions of the image to be resized and
    // grab the image size

	cv::Size dim;
    int h = sourceFrame.size().height;
	int w = sourceFrame.size().width;

    // if both the width and height are None, then return the
    // original image
    if ((width == -1) && (height==-1)){
        std::cerr << "You have to specify width or height!" << std::endl;
		exit(-1);
	}else{
		// check to see if the width is None
		if (width == -1){
		    // calculate the ratio of the height and construct the
		    // dimensions
		    float r = height / float(h);
		    dim = cv::Size(int(w * r), height);
		}
		// otherwise, the height is None
		else{
		    // calculate the ratio of the width and construct the
		    // dimensions
		    float ratio = width / float(w);
		    dim = cv::Size(width, int(h * ratio));
		}
		// resize the image
		cv::Mat resized(dim,CV_8UC3);
		cv::resize(sourceFrame, resized, resized.size(),0,0, CV_INTER_AREA);

		// return the resized image
		resultingFrame = resized.clone();
	}
}
void PlayerTracker::publishCameraTF(){
    static tf::TransformBroadcaster br;
    tf::StampedTransform playerTransform;

    // float current_servo_angle_radians = (M_PI/180) * current_servo_angle - M_PI/2;
    // //float rho = 0.003;
    // float rho = SERVO_ARM_LENGHT;

    // float x_p = rho * cos(current_servo_angle_radians); // x-coord of center arm
    // float y_p = rho * sin(current_servo_angle_radians); // y-coord of center arm

    // float x_displacement = CAM_DISPLACEMENT * cos(current_servo_angle_radians - M_PI/180/2); // x displacement of camera wrt center of servo arm
    // float y_displacement = CAM_DISPLACEMENT * sin(current_servo_angle_radians - M_PI/180/2); // y displacement of camera wrt center of servo arm

    // // NOT CONSIDERING CAMERA HORIZONTAL DISPLACEMENT
    // float x_cam = x_p + 0.07;
    // float y_cam = y_p;
    // float z_cam = 1.20;

    // // CONSIDERING HORIZONTAL DISPLACEMENT
    // // float x_cam = x_p + x_displacement;
    // // float y_cam = y_p + y_displacement;
    // // float z_cam = 1.20;

    // // float x_cam = x_p + x_displacement + 0.07;
    // // float y_cam = y_p + y_displacement;
    // // float z_cam = 1.20;

    // cam_pos.setX(x_cam);
    // cam_pos.setY(y_cam);
    // cam_pos.setZ(z_cam);

    // float theta = M_PI/4; // 45degs
    // float phi = (M_PI/180) * current_servo_angle - M_PI/2 + phi_target;
    // float x = rho * sin(theta) * cos(phi);
    // float y = rho * sin(theta) * sin(phi);
    float z = 1.20;


    
    tf::Transform framePlayerTransform;
    framePlayerTransform.setOrigin(tf::Vector3(0, 0, z));
    // framePlayerTransform.setOrigin(cam_pos);
    tf::Quaternion q;
    // q.setRPY(0, theta, current_servo_angle_radians);
    q.setRPY(0, 0, 0);
    framePlayerTransform.setRotation(q);
    ros::Time now = ros::Time::now();
    br.sendTransform(tf::StampedTransform(framePlayerTransform, now, "/base_link", "/onboard_link"));
}

/* Implements a "Region Growing algorithm", which is defined here in a "Breadth-first search" manner.
	sX --> Seed Pixel x value (columns == width)
	sY --> Seed Pixel y value (rows == height)
	threshold --> the value to be used in the call to "distanceFunction" method. If distance
    is less than threshold then recursion proceeds, else stops.*/
float PlayerTracker::segmentDepth(cv::Mat& inputFrame, cv::Mat& resultingFrame, int sX, int sY, float& threshold){
	
	long int nPixels = 0;                           			// segmented pixels counter variable.
	std::vector< std::vector<int> > reached;	       			// This is the binary mask for the segmentation.

	for (int i = 0; i < inputFrame.rows; i++){					//They are set to 0 at first. Since no pixel is assigned to the segmentation yet.
		reached.push_back(std::vector<int>(inputFrame.cols));
	}

	// Define the queue. NOTE: it is a BFS based algorithm.
	std::queue< std::pair<int,int> > seg_queue;

	// verify the depth value of the seed position.
	float &in_pxl_pos = inputFrame.at<float>(sY,sX);

    if(in_pxl_pos == 0){
        ROS_WARN_STREAM("THE SEED DEPTH VALUE IS ZERO!!!!!");
    }else{
    	//ROS_INFO_STREAM("THE SEED DEPTH VALUE NOT ZERO!!!!!");
        resultingFrame.at<float>(sY,sX) = 255;                     // add seed to output image.

        // Mark the seed as 1, for the segmentation mask.
    	reached[sY][sX] = 1;
        nPixels++;                                        // increase the pixel counter.

    	// init the queue witht he seed.
        seg_queue.push(std::make_pair(sY,sX));

        /* Loop over the frame, based on the seed adding a
            new pixel to the resultingFrame accordingly*/
    	while(!seg_queue.empty())
    	{
            /* pop values */
    		std::pair<int,int> s = seg_queue.front();
    		int x = s.second;
    		int y = s.first;
            seg_queue.pop();
            /* ... */

            /* The following "if" blocks analise the pixels incorporating them to the
                dst frame if they meet the threshold condition. */

    		// Right pixel
    		if((x + 1 < inputFrame.cols) && (!reached[y][x + 1]) &&
                distanceFunction(inputFrame.at<float>(sY,sX), inputFrame.at<float>(y, x + 1),
                threshold)){
                reached[y][x+1] = true;
                seg_queue.push(std::make_pair(y, x+1));
    			float &pixel = resultingFrame.at<float>(y,x+1);
                pixel = 255;
    			nPixels++;

    		}

    		//Below Pixel
    		if((y+1 < inputFrame.rows) && (!reached[y+1][x]) &&
                distanceFunction(inputFrame.at<float>(sY,sX), inputFrame.at<float>(y+1,x),
                threshold)){
    			reached[y + 1][x] = true;
    			seg_queue.push(std::make_pair(y+1,x));
    			resultingFrame.at<float>(y+1,x) = 255;
    			nPixels++;
    		}

    		//Left Pixel
    		if((x-1 >= 0) && (!reached[y][x-1]) &&
                distanceFunction(inputFrame.at<float>(sY,sX), inputFrame.at<float>(y,x-1),
                threshold)){
    			reached[y][x-1] = true;
    			seg_queue.push(std::make_pair(y,x-1));
    			resultingFrame.at<float>(y,x-1) = 255;
    			nPixels++;
    		}

    		//Above Pixel
    		if((y-1 >= 0) && (!reached[y - 1][x]) &&
                distanceFunction(inputFrame.at<float>(sY,sX), inputFrame.at<float>(y-1,x),
                threshold)){
    			reached[y-1][x] = true;
    			seg_queue.push(std::make_pair(y-1,x));
    			resultingFrame.at<float>(y-1,x) = 255;
    			nPixels++;
    		}

    		//Bottom Right Pixel
    		if((x+1 < inputFrame.cols) && (y+1 < inputFrame.rows) && (!reached[y+1][x+1]) &&
                distanceFunction(inputFrame.at<float>(sY,sX), inputFrame.at<float>(y+1,x+1),
                threshold)){
    			reached[y+1][x+1] = true;
    			seg_queue.push(std::make_pair(y+1,x+1));
    			resultingFrame.at<float>(y+1,x+1) = 255;
    			nPixels++;
    		}

    		//Upper Right Pixel
    		if((x+1 < inputFrame.cols) && (y-1 >= 0) && (!reached[y-1][x+1]) &&
                distanceFunction(inputFrame.at<float>(sY,sX), inputFrame.at<float>(y-1,x+1),
                threshold)){
    			reached[y-1][x+1] = true;
    			seg_queue.push(std::make_pair(y-1,x+1));
    			resultingFrame.at<float>(y-1,x+1) = 255;
    			nPixels++;
    		}

    		//Bottom Left Pixel
    		if((x-1 >= 0) && (y + 1 < inputFrame.rows) && (!reached[y+1][x-1]) &&
                distanceFunction(inputFrame.at<float>(sY,sX), inputFrame.at<float>(y+1,x-1),
                threshold)){
    			reached[y+1][x-1] = true;
    			seg_queue.push(std::make_pair(y+1,x-1));
    			resultingFrame.at<float>(y+1,x-1) = 255;
    			nPixels++;
    		}

    		//Upper left Pixel
    		if((x-1 >= 0) && (y-1 >= 0) && (!reached[y-1][x-1]) &&
                distanceFunction(inputFrame.at<float>(sY,sX), inputFrame.at<float>(y-1,x-1),
                threshold)){
    			reached[y-1][x-1] = true;
    			seg_queue.push(std::make_pair(y-1,x-1));
    			resultingFrame.at<float>(y-1,x-1) = 255;
                nPixels++;
    		}
    	}

        /* FROM THIS POINT ON: Initialization of supporting code for detecting the blob in the
		segmented frame. This is needed for drawing the minimum bounding rectangle
		for the detected blob, thus, enabling the "contraction index" feature
		calculation.*/
        std::vector<std::vector<cv::Point> > contours;
    	std::vector<cv::Vec4i> hierarchy;
        cv::Mat bwImage(inputFrame.size(),inputFrame.type());
        resultingFrame.convertTo(bwImage,CV_8U,255.0/(255-0));
    	cv::findContours(bwImage, contours, hierarchy,
                     CV_RETR_EXTERNAL,
                     CV_CHAIN_APPROX_SIMPLE);


        // Only proceed if at least one contour was found.
    	if (contours.size() > 0){

            cv::Mat erodeElement = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
            cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(8,8));
            cv::erode(resultingFrame, resultingFrame, erodeElement);
            cv::dilate(resultingFrame, resultingFrame, dilateElement);

            int largest_area=0;
    		int largest_contour_index=0;
    		

    		// find the largest contour in the mask, then use
    		// it to compute the minimum enclosing circle and
    		// centroid
    		for(int i=0; i < contours.size();i++){
    			// iterate through each contour.
                double coutorsArea = cv::contourArea(cv::Mat(contours[i]),false);  //  Find the area of contour
    			if( coutorsArea > largest_area){
    				largest_area=coutorsArea;
    				largest_contour_index=i;   //Store the index of largest contour
    			}
    		}

            //Calculate the minimum bounding box for the detected blob.
            cv::Rect rect = cv::boundingRect(contours[largest_contour_index]);
            cv::Point pt1, pt2;
            pt1.x = rect.x;
            pt1.y = rect.y;
            pt2.x = rect.x + rect.width;
            pt2.y = rect.y + rect.height;

            /* compute the Contraction index feature. It is the difference
            between the bounding rectangle area and the segmented object
            in the segmentation frame, normalized by
            the area of the rectangle. */
            cv::Mat roiSeg = cv::Mat(resultingFrame,rect);
            int roiSegarea = roiSeg.total();
            float ci = float(roiSegarea-nPixels)/float(roiSegarea);
			return ci;
            /* ... */

			////////////////
			// DEPRECATED //
			////////////////

            /*// APPLY COLOR TO THE FRAME
            std::vector<cv::Mat> tSegmentedInColor(3);                   // Used to print a colored
             														  	 //  segmented frame.
            cv::Mat black = cv::Mat::zeros(resultingFrame.rows,
             								resultingFrame.cols,
			 								resultingFrame.type());
            tSegmentedInColor.at(0) = black; 							 //for blue channel
            tSegmentedInColor.at(1) = resultingFrame;   				 //for green channel
            tSegmentedInColor.at(2) = black;  							 //for red channel
            
            cv::merge(tSegmentedInColor, segmentedColorFrame);
            
            //PRINTS THE MARKERS CALCULATED ABOVE.
            cv::circle(segmentedColorFrame, cv::Point(sX,sY),5,
             			cv::Scalar(0,0,255),CV_FILLED, 8,0);
            segmentedTarget = cv::Mat(segmentedColorFrame,rect);
            //cv::circle(segmentedTarget, cv::Point(topPoint,10),5,
            //			cv::Scalar(0,0,255),CV_FILLED, 8,0); // TAGGING THE POINT
            
            // Draws the rect in the segmentedColorFrame image
            cv::rectangle(segmentedColorFrame, pt1, pt2, cv::Scalar(255,255,255), 2,8,0);// the selection white rectangle
            

			cv::putText(segmentedColorFrame,
					std::to_string((int)blobCenter.x),			// The text to be printed, in this case, CI.
					cv::Point(blobCenter.x,blobCenter.y), 		// Coordinates
					cv::FONT_HERSHEY_COMPLEX_SMALL, 			// Font
						0.5, 									// Scale. 2.0 = 2x bigger
						cv::Scalar(255,255,255),				// Color
						1 										// Thickness
						); 										// Anti-alias
            
            // put the CI value to frame
            cv::putText(segmentedColorFrame,
            			std::to_string(ci),				// The text to be printed, in this case, CI.
						cv::Point(rect.x,rect.y-5), 	// Coordinates
						cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
						0.9, 							// Scale. 2.0 = 2x bigger
			 			cv::Scalar(255,255,255),		// Color
						1 								// Thickness
            			); 								// Anti-alias*/
		/////////////////////////////////////////////////////////////////////////////////
		}
    }
}


int main(int argc, char** argv){
    
    ros::init(argc, argv, "realsense_tracking");

    PlayerTracker player_tracker;
    
    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, onShutdown);
  

	// if (show_frame){
	// 	cv::namedWindow("view", 1);
	// 	cv::namedWindow("seg", 1);
	// 	cv::startWindowThread();
	// }

    // Image transport handle
    image_transport::ImageTransport it(player_tracker.nh);

    // Create a subscriber.
    // Set queue size to 1 because generating a queue here will only pile up images and delay the output by the amount of queued images
    image_transport::SubscriberFilter subscriber_depth;
    image_transport::SubscriberFilter subscriber_image;

    subscriber_depth.subscribe(it, player_tracker.topic_depth_image.c_str(),1);
    subscriber_image.subscribe(it, player_tracker.topic_color_image.c_str(),1);
	
    //The real queue size for synchronisation is set here.
    sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy(QUEUE_SIZE);
    MySyncPolicy.setAgePenalty(1000); //set high age penalty to publish older data faster even if it might not be correctly synchronized.


    // Create synchronization policy. Here: async because time stamps will never match exactly
    const sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> MyConstSyncPolicy = MySyncPolicy;
    Synchronizer< sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> > sync(MyConstSyncPolicy,
                                                                                       subscriber_depth,
                                                                                       subscriber_image);
    // Register one callback for all topics
    sync.registerCallback(boost::bind(&PlayerTracker::callback,&player_tracker, _1, _2));
    

    ros::Rate rate(30);
    
    ROS_INFO("Tracking player...");
    
    while(ros::ok() && !is_exit){
        player_tracker.publishCameraTF();
	    ros::spinOnce();
        rate.sleep() ;

    }

    return 0;
}
