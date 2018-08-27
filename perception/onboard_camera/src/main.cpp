/* main.cpp -- Identifies the bearing angle of the robot w.r.t. towers
 *
 * Copyright (C) 2018 Ewerton Lopes
 *
 * LICENSE: This is a free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License (LGPL v3) as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version. This code is distributed in the hope
 * that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU Lesser General Public License (LGPL v3): http://www.gnu.org/licenses/.
 */

/* Built-in includes */
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
#include <math.h>

/* ROS related includes */
#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

/* Boost includes */
#include <boost/circular_buffer.hpp>

/* OpenCV related includes */
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

/* Eigen */
#include <eigen3/Eigen/Dense>

/* Local includes */
#include "utils.h"
#include "OnboardCamera.h"

#define QUEUE_SIZE 5
#define RATE 30

bool is_exit = false;
bool is_shutdown = false;

OnboardCamera::OnboardCamera(): it(nh), pts(QUEUE_SIZE), angle_when_lost_player(15){

    is_exit = false;
    is_shutdown = false;
    show_frame = false;

    vel_angle = 90; /* Deprecated*/

    float delta_t = (1./RATE);
    float delta_t2 = pow(delta_t,2)/2;

    // Kalman filter matrices
    x << 0.0, 0.0;
    
    u << 0.0, 0.0;
    
    // Measurement Function 2 states - 1 observed (angle)
    H << 1.0, 0.0;

    // Identity Matrix
    I << 1.0, 0.0,                
         0.0, 1.0;     

    // Measurement Uncertainty
    R << 0.1;

    // Initial Uncertainty
    P << 1.0, 0.0,            
         0.0, 1.0;
         
    // Transition Matrix
    F << 1.0,delta_t,                       // angle
         0.0,delta_t;                       // angular speed

    // Process Noise Matrix
    Q << 0.1, 0.0,                   
         0.0, 0.1;

    nh.getParam("camera_namespace", cam_ns);
    nh.getParam("show_frame", show_frame);
    nh.getParam("color_frame", color_frame_name);
    nh.getParam("cam_width", cam_width);
    nh.getParam("cam_height", cam_height);
    nh.getParam("width_fov", width_fov);
    nh.getParam("height_fov", height_fov);
    nh.getParam("seg_threshold", seg_threshold);
    nh.getParam("h_min", hMin);
    nh.getParam("s_min", sMin);
    nh.getParam("v_min", vMin);
    nh.getParam("h_max", hMax);
    nh.getParam("s_max", sMax);
    nh.getParam("v_max", vMax);

    
    topic_color_image = cam_ns + color_frame_name;

    sub = it.subscribe(topic_color_image.c_str(), 
                       1,
                       &OnboardCamera::imageCallback, this);


    sub_servo_angle = nh.subscribe("/arduino/current_servo_angle", 
                             1,
                             &OnboardCamera::servoAngleCallback, this);

    pub_servo_angle = nh.advertise<std_msgs::Int16>("/onboard_cam/rotate",1);

    ROS_INFO_STREAM("Color image topic: " << topic_color_image);
	ROS_INFO_STREAM("Show frames: " << (show_frame ? "True" : "False"));

  	tfListener = new tf::TransformListener();

}


void OnboardCamera::publishKalman(){
    ROS_INFO_STREAM(x(0,0));
}

void OnboardCamera::kalmanPredict(){
    x = F * x + u;
    P = F*P*F.transpose() + Q;
}

void OnboardCamera::kalmanUpdate(float angle){
    Eigen::Matrix<float, 1, 1> Z;
    Z << angle;
    auto Y =  Z.transpose() - (H*x);
    auto S = ((H*P) * H.transpose()) + R;
    auto K = (P * H.transpose()) * S.inverse();
    x = x + (K * Y);
    P = (I - (K*H)) * P;
}

void OnboardCamera::imageCallback(const sensor_msgs::ImageConstPtr &image){

    cv_bridge::CvImagePtr cv_rgb_ptr;
    
    try{
        // Get image as matrix
        cv_rgb_ptr = cv_bridge::toCvCopy(image);
    }catch (cv_bridge::Exception& e){
        ROS_ERROR("%s",e.what());
        return;
    }

    cv::Mat rgbmat = cv_rgb_ptr->image;
    
    int found_blob = findColorBlob(rgbmat, blob_center);

    /* THIS LOOP COMPUTES A REGION OF INTEREST (A CIRCLE) BASED ON THE mainCenter VARIABLE COMPUTED BY
    THE findColorBlob METHOD. THE IDEA IS THEN TO ASSESS THE MEAN DISTANCE (PIXEL VALUES)
    DEFINED IN THIS AREA AND THUS OBTAIN THE DISTANCE FEATURE. THE SAME LOOP ALSO CALL THE
    segmentDepth METHOD IN ORDER TO OBTAIN THE CONTRACTION INDEX FEATURE.*/


    if (found_blob){
        // compute the angle of the blob_center with respect to the camera.
        // this uses spherical coordinate system in order to retrieve the phi parameter (angle)
        // details here: https://en.wikipedia.org/wiki/Spherical_coordinate_system

        float phi = (0.5 - blob_center.x / cam_width) * width_fov;                // yaw
        phi_target = phi;
        ROS_DEBUG_STREAM("Yaw: " << phi);


        pts.push_front(phi);
        
        float mean = 0;
        for(int i=0; i < pts.size() ; i++){
            mean += pts[i] / pts.size();
        }
        

        mean = mean * (180/M_PI);

        if (abs(mean) > 1){

            kalmanUpdate(mean);
            kalmanPredict(); // Perform Kalman prediction

            ROS_INFO_STREAM("Angle mismatch: " << mean);
            ROS_WARN_STREAM("Angle mismatch (KALMAN): " << x(0,0));
            std_msgs::Int16 angle_msg = std_msgs::Int16();
            angle_msg.data = x(0,0)/2;
            pub_servo_angle.publish(angle_msg);

            float theta = M_PI / 2 - (0.5 - blob_center.y / cam_height) * height_fov;   // pitch

            float rho = 1;

            float x = rho * sin(theta) * cos(phi);
            float y = rho * sin(theta) * sin(phi);
            float z = rho * cos(theta);
            
            cam_pos.setX(x);
            cam_pos.setY(y);
            cam_pos.setZ(z);
            
            ROS_DEBUG_STREAM("Blob is at (" << phi << ", " << theta << ") deg(s) from the camera.");


            angle_when_lost_player *= mean/mean;

            // ROS_DEBUG("rho:\t%.2f\tphi:\t%.2f°\ttheta:\t%.2f°", rho, phi*180/M_PI, theta*180/M_PI);
            // ROS_DEBUG("x:\t%.2f\ty:\t%.2f\tz:\t%.2f", pixel_sph_coord(0,0), pixel_sph_coord(1,0), pixel_sph_coord(2,0));
        }
        
        // // TF-Broadcaster
        static tf::TransformBroadcaster br;
        tf::StampedTransform playerTransform;
        
        tf::Transform framePlayerTransform;
        framePlayerTransform.setOrigin(cam_pos);
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        framePlayerTransform.setRotation(q);
        ros::Time now = ros::Time::now();
        br.sendTransform(tf::StampedTransform(framePlayerTransform, now, "/onboard_link", "/cam_target_link"));

        
    }
    else{

        ROS_DEBUG_STREAM("Blob not found, rotating with a constant of " << angle_when_lost_player << "degs..");
        std_msgs::Int16 angle_msg = std_msgs::Int16();
        
        if (current_servo_angle == 0){
            angle_when_lost_player = +15;
        }else if (current_servo_angle == 180) {
            angle_when_lost_player = -15;
        }

        angle_msg.data = angle_when_lost_player;
        pub_servo_angle.publish(angle_msg);

    }

    
    //publishKalman(); // publish Kalman filtered player tf.
        
	if (show_frame){
		cv::imshow("view", rgbmat);
		int key = cv::waitKey(30);
	}
}


void OnboardCamera::publishCameraTF(){
    static tf::TransformBroadcaster br;
    tf::StampedTransform playerTransform;

    float rho = 0.003;
    float theta = M_PI/4; // 45degs
    float phi = (M_PI/180) * current_servo_angle - M_PI/2 + phi_target;
    float x = rho * sin(theta) * cos(phi);
    float y = rho * sin(theta) * sin(phi);
    float z = 1.20;
    
    tf::Transform framePlayerTransform;
    framePlayerTransform.setOrigin(tf::Vector3(0, 0, z));
    tf::Quaternion q;
    q.setRPY(0, theta, phi);
    framePlayerTransform.setRotation(q);
    ros::Time now = ros::Time::now();
    br.sendTransform(tf::StampedTransform(framePlayerTransform, now, "/base_link", "/onboard_link"));
}

// Connection callback that unsubscribes from the tracker if no one is subscribed.
void OnboardCamera::connectCallback(image_transport::SubscriberFilter &sub_col,
                     image_transport::SubscriberFilter &sub_dep,
                     image_transport::ImageTransport &it) {
                     
    ROS_DEBUG("Onboard camera node: New subscribers. Subscribing.");
    sub_col.subscribe(it,sub_col.getTopic().c_str(),1);
    sub_dep.subscribe(it,sub_dep.getTopic().c_str(),1);

}

/** DEPRECATED
 * Calculates de angle offset based on the direction of the robot's velocity vector.
 */
void OnboardCamera::velCallback(const geometry_msgs::TwistPtr &msg){
   if (msg->linear.y != 0 || msg->linear.x != 0){
        vel_angle = (M_PI/2) + atan2(msg->linear.y, msg->linear.x);
        if (vel_angle > 0 && vel_angle < M_PI){
            std_msgs::Int16 new_servo_angle;
            new_servo_angle.data = vel_angle  * (180/M_PI);
            ROS_DEBUG_STREAM("vel_angle: " << vel_angle * (180/M_PI));
            ROS_DEBUG_STREAM("new angle: " << new_servo_angle.data);
            pub_servo_angle.publish(new_servo_angle);
        }
   }
}

void OnboardCamera::servoAngleCallback(const std_msgs::Int16Ptr &msg){
   current_servo_angle = msg->data;
   ROS_DEBUG_STREAM("Current servo angle: " << msg->data);
}


/* Locate largest color blob on a RGB image. */
int OnboardCamera::findColorBlob(cv::Mat& srcFrame,  cv::Point2f &blob_center){
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

void OnboardCamera::resize(cv::Mat& sourceFrame, cv::Mat& resultingFrame, int width, int height){
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


int main(int argc, char** argv){
    
    ros::init(argc, argv, "onboard_camera_node");

    OnboardCamera onboard_camera;

    ros::Rate rate(30);
    
    while(ros::ok() && !is_exit){
        onboard_camera.publishCameraTF();
        rate.sleep() ;
        ros::spinOnce();
    }
    return 0;
}
