/* main.cpp
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
#include "OnboardCameraRealSense.h"

#define QUEUE_SIZE 5
#define RATE 30

#define SERVO_ARM_LENGHT 0.085 // 8.5 cm from servo train
#define CAM_DISPLACEMENT 0.015 // 1.5 cm from center
#define ANGULAR_DISPLACEMENT -6 // servo is rotated 15 degs more than 90 at begin

bool is_exit = false;
bool is_shutdown = false;

OnboardCamera::OnboardCamera(): it(nh), distance_buffer(BUFFER_DIST_SIZE), pts(QUEUE_SIZE), angle_when_lost_player(15){

    mean_distance = 0;    			// distance feature
    pr_mean_distance;     			// variable for distance at time t-1.
    ci = 0;               			// contraction index
    pr_ci = 0;            			// variable for ci at time t-1.
    is_player_missing = false;     // a flag for the player presence.
    is_exit = false;
    is_shutdown = false;
    show_frame = true;
    
    vel_angle = 90; /* Deprecated*/

    tfListener = new tf::TransformListener();

    float delta_t = (1./RATE);
    float delta_t2 = pow(delta_t,2)/2;

    // Kalman filter matrices
    x << 0,0,0,0,0,0;

    u << 0,0,0,0,0,0;

    // measurement function 6 states - 2 observed (angle and distance)
    H << 1,0,0,0,0,0,   
         0,1,0,0,0,0;  

    // identity matrix
    I << 1,0,0,0,0,0,            
         0,1,0,0,0,0,
         0,0,1,0,0,0,
         0,0,0,1,0,0,
         0,0,0,0,1,0,
         0,0,0,0,0,1;     

    // measurement uncertainty (2 uncorrelated measures with uncertainty)
    R << 0.1,0,
         0,0.3;

    // initial uncertainty
    P << 1,0,0,0,0,0,            
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

    // process noise matrix
    Q << 0.1,0,0,0,0,0,        
         0,0.1,0,0,0,0,
         0,0,0.01,0,0,0,
         0,0,0,0.01,0,0,
         0,0,0,0,0.001,0,
         0,0,0,0,0,0.001;

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


    sub_servo_angle = nh.subscribe("/arduino/current_servo_angle", 
                                    1,
                                    &OnboardCamera::servoAngleCallback, this);

    pub_servo_angle = nh.advertise<std_msgs::Int16>("/onboard_cam/rotate",1);

    ROS_INFO_STREAM("Color image topic: " << topic_color_image);
    ROS_INFO_STREAM("Color depth topic: " << topic_depth_image);
	ROS_INFO_STREAM("Show frames: " << (show_frame ? "True" : "False"));

}


void OnboardCamera::kalmanPredict(){
    x = F * x + u;
    P = F*P*F.transpose() + Q;
}

void OnboardCamera::kalmanUpdate(float angle, float distance){
    Eigen::Matrix<float, 1, 2> Z;
    Z << angle, distance;
    auto Y =  Z.transpose() - (H*x);
    auto S = ((H*P) * H.transpose()) + R;
    auto K = (P * H.transpose()) * S.inverse();
    x = x + (K * Y);
    P = (I - (K*H)) * P;
}


Eigen::Matrix<float, 3, 1> OnboardCamera::getSphericalCoordinate(float rho, float theta, float phi){
    float x = rho * sin(theta) * cos(phi);
    float y = rho * sin(theta) * sin(phi);
    float z = rho * cos(theta);
    Eigen::Matrix<float, 3, 1> m;
    m << x,y,z;
    return m;
}

void OnboardCamera::publishKalman(){
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


void OnboardCamera::callback(const sensor_msgs::ImageConstPtr &depth, const sensor_msgs::ImageConstPtr &image){

    ROS_WARN("iMAGE Received");
    
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

    cv::Mat segmat = cv::Mat::zeros(depmat.size(), depmat.type());
    segmat.convertTo(segmat,CV_32FC1,  0.001);

    /* THIS LOOP COMPUTES A REGION OF INTEREST (A CIRCLE) BASED ON THE mainCenter VARIABLE COMPUTED BY
    THE findColorBlob METHOD. THE IDEA IS THEN TO ASSESS THE MEAN DISTANCE (PIXEL VALUES)
    DEFINED IN THIS AREA AND THUS OBTAIN THE DISTANCE FEATURE. THE SAME LOOP ALSO CALL THE
    segmentDepth METHOD IN ORDER TO OBTAIN THE CONTRACTION INDEX FEATURE.*/


    int offset = 5; // an offset w.r.t the blob_center. We use it to calc an area around the blob_center.


    if (found_blob){
        // compute the angle of the blob_center with respect to the camera.
        // this uses spherical coordinate system in order to retrieve the phi parameter (angle)
        // details here: https://en.wikipedia.org/wiki/Spherical_coordinate_system


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
    
        cv::Scalar distance = cv::mean(roi);        // compute mean value of the region of interest.
                                                   // RECALL: the pixels correspond to distance in mm.

        mean_distance = distance[0] / 1000.0f;     // compute distance (in meters)

        // perform segmentation in order to get the contraction index featue.
        //ci = segmentDepth(depmat, segmat, blob_center.x, blob_center.y, seg_threshold);    
        
        // Defining rho, phi and theta
        float rho = mean_distance;
        float phi = (0.5 - blob_center.x / cam_width) * width_fov;                // yaw
        float theta = M_PI / 2 - (0.5 - blob_center.y / cam_height) * height_fov;
        

        // getting spherical coordinate
        Eigen::Matrix<float, 3, 1> pixel_sph_coord = getSphericalCoordinate(rho,theta,phi);    // return column vector (newx,newy,newz) pixel pos.

        phi_target = phi;

        // pts.push_front(phi);
        
        // float mean_phi = 0;
        // for(int i=0; i < pts.size() ; i++){
        //     mean_phi += pts[i] / pts.size();
        // }

        // KALMAN UPDATE
        distance_buffer.push_front(mean_distance);          // push into a simple low-pass filter.
        float mean_of_buffer = accumulate(distance_buffer.begin(), distance_buffer.end(), 0.0)/distance_buffer.size();
        pr_blob_center = blob_center;
        kalmanUpdate(phi, mean_distance);

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
        
        if (fabs(phi * (180/M_PI)) > 10){
            ROS_INFO_STREAM("Angle mismatch: " << phi);
            ROS_WARN_STREAM("Angle mismatch (KALMAN): " << x(0,0));
            std_msgs::Int16 angle_msg = std_msgs::Int16();
            angle_msg.data = x(0,0) * (180/M_PI);
            pub_servo_angle.publish(angle_msg);
        }

        target_pos.setX(pixel_sph_coord(0,0));
        target_pos.setY(pixel_sph_coord(1,0));
        target_pos.setZ(pixel_sph_coord(2,0));
    } 

    kalmanPredict(); // Perform Kalman prediction
    publishKalman(); // publish Kalman filtered player tf.
        
	if (show_frame){
		cv::imshow("view", rgbmat);
		int key = cv::waitKey(30);
	}
}


void OnboardCamera::publishCameraTF(){
    static tf::TransformBroadcaster br;
    tf::StampedTransform playerTransform;

    float current_servo_angle_radians = (M_PI/180) * current_servo_angle - M_PI/2;
    // //float rho = 0.003;
    // float rho = SERVO_ARM_LENGHT;

    // float x_p = rho * cos(current_servo_angle_radians); // x-coord of center arm
    // float y_p = rho * sin(current_servo_angle_radians); // y-coord of center arm

    // float x_displacement = CAM_DISPLACEMENT * cos(current_servo_angle_radians - M_PI/180/2); // x displacement of camera wrt center of servo arm
    // float y_displacement = CAM_DISPLACEMENT * sin(current_servo_angle_radians - M_PI/180/2); // y displacement of camera wrt center of servo arm

    // NOT CONSIDERING CAMERA HORIZONTAL DISPLACEMENT
    // float x_cam = x_p + 0.07;
    // float y_cam = y_p;
    // float z_cam = 0.74;
    float x_cam = 0;
    float y_cam = 0;
    float z_cam = 0.74;

    // CONSIDERING HORIZONTAL DISPLACEMENT
    // float x_cam = x_p + x_displacement;
    // float y_cam = y_p + y_displacement;
    // float z_cam = 1.20;

    // float x_cam = x_p + x_displacement + 0.07;
    // float y_cam = y_p + y_displacement;
    // float z_cam = 1.20;

    cam_pos.setX(x_cam);
    cam_pos.setY(y_cam);
    cam_pos.setZ(z_cam);

    float theta = 0.349066; //M_PI/4; // 45degs
    // float phi = (M_PI/180) * current_servo_angle - M_PI/2 + phi_target;
    // float x = rho * sin(theta) * cos(phi);
    // float y = rho * sin(theta) * sin(phi);
    // float z = 1.20;


    
    tf::Transform framePlayerTransform;
    //framePlayerTransform.setOrigin(tf::Vector3(0, 0, z));
    framePlayerTransform.setOrigin(cam_pos);
    tf::Quaternion q;
    q.setRPY(0, theta, current_servo_angle_radians);
    framePlayerTransform.setRotation(q);
    ros::Time now = ros::Time::now();
    br.sendTransform(tf::StampedTransform(framePlayerTransform, now, "/base_link", "/onboard_link"));
}

// Connection callback that unsubscribes from the tracker if no one is subscribed.
void OnboardCamera::connectCallback(image_transport::SubscriberFilter &sub_col,image_transport::SubscriberFilter &sub_dep,image_transport::ImageTransport &it) {
                     
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
   current_servo_angle = msg->data + ANGULAR_DISPLACEMENT; //* (msg->data / 180.0);
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
            cv::circle(frame, cv::Point(320, int(temp_center.y)), 5, cv::Scalar(255, 0, 0), -1);
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
    
    ros::init(argc, argv, "realsense_tracking");

    OnboardCamera player_tracker;

    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    //signal(SIGINT, onShutdown);


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
    message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy(QUEUE_SIZE);
    MySyncPolicy.setAgePenalty(1000); //set high age penalty to publish older data faster even if it might not be correctly synchronized.


    // Create synchronization policy. Here: async because time stamps will never match exactly
    const message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> MyConstSyncPolicy = MySyncPolicy;
    message_filters::Synchronizer< message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> > sync(MyConstSyncPolicy,
                                                                                        subscriber_depth,
                                                                                        subscriber_image);
    // Register one callback for all topics
    sync.registerCallback(boost::bind(&OnboardCamera::callback,&player_tracker, _1, _2));


    ros::Rate rate(30);

    ROS_INFO("Tracking player...");

    while(ros::ok()){
        player_tracker.publishCameraTF();
        ros::spinOnce();
        rate.sleep() ;

    }

    return 0;
}
