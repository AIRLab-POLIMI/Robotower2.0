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
#include <boost/circular_buffer.hpp>
#include <math.h>       /* sqrt */

/* ROS related includes */
#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <robogame_kinectfeatures_extractor/kinect_feat.h>
#include<std_msgs/Int32.h>
/* ... */

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

#define KIN_WIDTH 512
#define KIN_HEIGHT 424

#define WIDTH_FOV 70.6*M_PI/180;
#define HEIGHT_FOV 60.0*M_PI/180;

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
// previous feature values
float previousDistance;         // variable for distance at time t-1.
float previousCI;               // variable for ci at time t-1.
// ..
bool isPlayerMissing;              // a flag for the player presence.

boost::circular_buffer<cv::Point2f> pts(TRAIL_BUFFER_SIZE);    // The blob location history

cv::Mat segmentedColorFrame;    // the segmented color frame.
cv::Mat segmentedTarget;

using namespace std;
using namespace cv;

// Global tf listener pointer
tf::TransformListener* tfListener;

ros::Publisher player_global_pose_pub;
ros::Publisher player_local_pose_pub;

enum Processor { cl, gl, cpu };         // libfreenect2 enum.

bool protonect_shutdown = false;        // Whether the running application should shut down.

/* Interruption handler function. In this case, set the variable that controls the
 frame aquisition, breaking the loop, cleaning variables and exit the program elegantly*/
void sigint_handler(int s){
    protonect_shutdown = true;
}

int main(int argc, char** argv)
{
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "publish_kinect");
    ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);
	image_transport::Publisher imgPub = it.advertise("camera/image", 1);
	image_transport::Publisher segmentationPub = it.advertise("camera/image/player_segmented", 1);
	//image_transport::Publisher imgPub = it.advertise("camera/image/depth", 1);

	//Create a publisher object.
    ros::Publisher pub = nh.advertise<robogame_kinectfeatures_extractor::kinect_feat>("kinect_features",1000);
    ros::Publisher position_pub = nh.advertise<std_msgs::Int32>("robogame/player_x_position",1000);

    player_global_pose_pub = nh.advertise<geometry_msgs::PoseStamped> ("robogame/player_global_position",1000);
	player_local_pose_pub  = nh.advertise<geometry_msgs::PoseStamped> ("robogame/player_local_position",1000);

	tfListener = new tf::TransformListener();

    std::cout << "Connecting to sensor..." << std::endl;

    //! libfreenect2 [context]
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = nullptr;
    libfreenect2::PacketPipeline *pipeline = nullptr;
    //! [context]

    //! [discovery] of sensor --  Kinect, in this case.
    if(freenect2.enumerateDevices() == 0){
        std::cout << "no depth camera device connected!" << std::endl;
        return -1;
    }

    // getting the sensor serial.
    string serial = freenect2.getDefaultDeviceSerialNumber();

    std::cout << "SERIAL: " << serial << std::endl;
    //! [discovery]

    /* LIBFREENECT2 INITIALIZATION CODE - PROCESSOR TYPE DEFINITION*/
    int depthProcessor = Processor::cl;
    if(depthProcessor == Processor::cpu){
        if(!pipeline)
            //! [pipeline]
            pipeline = new libfreenect2::CpuPacketPipeline();
            //! [pipeline]
    }else if(depthProcessor == Processor::gl){
        #ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
                if(!pipeline)
                    pipeline = new libfreenect2::OpenGLPacketPipeline();
        #else
                std::cout << "OpenGL pipeline is not supported!" << std::endl;
        #endif
            } else if (depthProcessor == Processor::cl) {
        #ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
                if(!pipeline)
                    pipeline = new libfreenect2::OpenCLPacketPipeline();
        #else
                std::cout << "OpenCL pipeline is not supported!" << std::endl;
        #endif
    }

    if(pipeline){
        //! [open]
        dev = freenect2.openDevice(serial, pipeline);
        //! [open]
    }else {
        dev = freenect2.openDevice(serial);
    }

    if(dev == 0){
        std::cout << "failure opening device!" << std::endl;
        return -1;
    }
    /* ---- */

    signal(SIGINT, sigint_handler);             // Set interruption handler
    protonect_shutdown = false;                 // Define value of the loop control variable

    //! [listeners]
    libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color |
                                                  libfreenect2::Frame::Depth |
                                                  libfreenect2::Frame::Ir);
    libfreenect2::FrameMap frames;

    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);
    //! [listeners]

    //! [start]
    dev->start();
    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
    //! [start]

    //! [registration setup]
    libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4), depth2rgb(1920, 1080 + 2, 4); // check here (https://github.com/OpenKinect/libfreenect2/issues/337) and here (https://github.com/OpenKinect/libfreenect2/issues/464) why depth2rgb image should be bigger
    //! [registration setup]

    // Opencv frame definition. They are the containers from which frames are writen to and exihibit.
    cv::Mat original_rgbmat, original_depthmat, original_regframe, rgbmat, depthmat, regframe, depthmatUndistorted, irmat, rgbd, rgbd2, depthAndRgb, undistortedFrame;
    int count = 0;

	// Define object to save frame to video. (used for data tagging purposes)
	//cv::VideoWriter videoWriter("/home/airlab/catkin_ws/src/robogame_kinectfeatures_extractor/videos/out.avi",CV_FOURCC('M', 'J', 'P', 'G'),30.0,cv::Size(undistorted.height,undistorted.width),true);

    /* Create the blob detection image panel together with the
        sliders for run time adjustments. */
    cv::namedWindow("mask", 1);

    cv::createTrackbar("hMin", "mask", &hMin, 256);
    cv::createTrackbar("sMin", "mask", &sMin, 256);
    cv::createTrackbar("vMin", "mask", &vMin, 256);
    cv::createTrackbar("hMax", "mask", &hMax, 256);
    cv::createTrackbar("sMax", "mask", &sMax, 256);
    cv::createTrackbar("vMax", "mask", &vMax, 256);
    /* ---- */

    /* FEATURE VARIABLES */
    float meanDistance = 0;         // distance feature
    float ci = 0;                   // contraction index

    // ROS loop at 2Hz until the node is shut down.
    ros::Rate rate(40);

    //! [loop start] -- DATA AQUISITION
    while(!protonect_shutdown && ros::ok())
    {
        /* Kinect data aquisition */
        listener.waitForNewFrame(frames);
        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

        //Apply and read the registration video stream
        registration->apply(rgb,depth,&undistorted,&registered);
        /* ---- */

        /* Save the aquired data to the cv::mat containers. */
        cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(original_rgbmat);
        cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(original_depthmat);
        cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(original_regframe);


		cv::flip(original_rgbmat,   rgbmat, 1);
		cv::flip(original_depthmat, depthmat,1);
		cv::flip(original_regframe, regframe,1);

		/*converting the rgb into grayscale.. Why? compression purposes*/
		//cv::Mat im_gray;
		//cvtColor(rgbmat,im_gray,CV_RGB2GRAY);
		//resize(im_gray, im_gray, cv::Size(512,424), 2.28, 2.28, cv::INTER_LANCZOS4);
        resize(rgbmat, rgbmat, cv::Size(512,424));

		/* Publishes the image. NOTE:: Here, the rgb image  is in fact a bgr image (bgra8, i.e., CV8U4).
        see http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages */
		//sensor_msgs::ImagePtr imgmsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", im_gray).toImageMsg();
		sensor_msgs::ImagePtr imgmsg = cv_bridge::CvImage(std_msgs::Header(), "bgra8", rgbmat).toImageMsg();
		imgPub.publish(imgmsg);
		//imgPub.publish(depthmsg);


        /* Search for the color blob representing the human. */
        trackUser(regframe);

        /* Supporting frames*/
        cv::Mat original_undistortedFrame = cv::Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data);
        cv::flip(original_undistortedFrame,undistortedFrame,1);
        cv::Mat segmat = Mat::zeros(undistortedFrame.size(), undistortedFrame.type());
        segmentedColorFrame = Mat::zeros(undistortedFrame.size(), CV_8UC3);
        segmentedTarget = Mat::zeros(undistortedFrame.size(), CV_8UC3);

//        /* THIS LOOP COMPUTES A REGION OF INTEREST (A CIRCLE) BASED ON THE mainCenter VARIABLE COMPUTED BY
//            THE trackUser METHOD. THE IDEA IS THEN TO ASSESS THE MEAN DISTANCE (PIXEL VALUES)
//            DEFINED IN THIS AREA AND THUS OBTAIN THE DISTANCE FEATURE. THE SAME LOOP ALSO CALL THE
//            segmentDepth METHOD IN ORDER TO OBTAIN THE CONTRACTION INDEX FEATURE.*/
//        if ((blobCenter.x != -1000) && (blobCenter.x != 0)){
//            int radius = 5;
//
//            //get the Rect containing the circle:
//            cv::Rect r(blobCenter.x-radius, blobCenter.y-radius, radius*2,radius*2);
//            // obtain the image ROI:
//            cv::Mat roi(undistortedFrame, r);
//
//            // make a black mask, same size:
//            cv::Mat maskROI(roi.size(), roi.type(), cv::Scalar::all(0));
//
//            // with a white, filled circle in it:
//            cv::circle(maskROI, cv::Point(radius,radius), radius, cv::Scalar::all(255), -1);
//
//            // combine roi & mask:
//            cv::Mat roiArea = roi & maskROI;
//            // -------
//
//            cv:Scalar m = cv::mean(roi);        // compute mean value of the region of interest.
//                                            //    RECALL: the pixels correspond to distance in mm.
//            meanDistance = m[0] / 1000.0f;  // compute distance (in meters)
//
//            /* perform segmentation in order to get the contraction index featue.
//                The result will be saved in ci variable*/
//            segmentDepth(undistortedFrame, segmat, blobCenter.x, blobCenter.y, ci, 300);
//        }
//
//        /* A LOOP TO PRINT THE DISTANCE AND HISTORY TRACE IN THE DEPTH FRAME. IT JUST SERVES AS A
//         VISUAL AID OF WHAT IS GOING ON.*/
//    	// for (int i=1; i < (pts.size()-1); i++){
//    	// 	// if either of the tracked points are None, ignore
//    	// 	// them
//        //
//    	// 	cv::Point2f ptback = pts[i - 1];
//    	// 	cv::Point2f pt = pts[i];
//    	// 	if ((ptback.x == -1000) or (pt.x == -1000)){
//        //         continue;
//    	// 	}
//        //
//    	// 	// otherwise, compute the thickness of the line and
//    	// 	// draw the connecting lines
//    	// 	int thickness = int(sqrt(TRAIL_BUFFER_SIZE / float(i + 1)) * 2.5);
//    	// 	line(undistortedFrame, pts[i - 1], pts[i], cv::Scalar(0, 0, 255), thickness);
//        //     // Write distance
//        //     cv::putText(undistortedFrame,
//        //     std::to_string(meanDistance),
//        //     cv::Point((512/2)-60,85), // Coordinates
//        //     cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
//        //     1.0, // Scale. 2.0 = 2x bigger
//        //     cv::Scalar(255,255,255), // Color
//        //     1 // Thickness
//        //     ); // Anti-alias
//    	// }




        ////////////////// NEW CODE ////////////////////

        /* THIS LOOP COMPUTES A REGION OF INTEREST (A CIRCLE) BASED ON THE mainCenter VARIABLE COMPUTED BY
        	   THE trackUser METHOD. THE IDEA IS THEN TO ASSESS THE MEAN DISTANCE (PIXEL VALUES)
        	   DEFINED IN THIS AREA AND THUS OBTAIN THE DISTANCE FEATURE. THE SAME LOOP ALSO CALL THE
        	   segmentDepth METHOD IN ORDER TO OBTAIN THE CONTRACTION INDEX FEATURE.*/

		int radius = 5;

		if ((blobCenter.x != -1000) && (blobCenter.x != 0) && blobCenter.x >= 5*radius && blobCenter.y >= 5*radius &&
				blobCenter.x < depthmat.size().width - 5*radius && blobCenter.y < depthmat.size().height - 5*radius ){
		// if ((blobCenter.x != -1000) && (blobCenter.x != 0)){

			bool skip = false;

			//get the Rect containing the circle:
			cv::Rect r(blobCenter.x-radius, blobCenter.y-radius, radius*2, radius*2);
			cv::Mat roi, roiArea;
			try{
				// obtain the image ROI:
				depthmat.convertTo(depthmat, CV_32F,  1.0);
				roi = cv::Mat(depthmat, r);

				// make a black mask, same size:
				cv::Mat maskROI(roi.size(), roi.type(), cv::Scalar::all(0));

				// with a white, filled circle in it:
				cv::circle(maskROI, cv::Point(radius,radius), radius, cv::Scalar::all(255), -1);

				// combine roi & mask:
				roiArea = roi & maskROI;
				// -------
			} catch (cv::Exception ex) {
				ROS_ERROR("%s",ex.what());
				skip = true;
			}

			if (!skip){
		   ROS_INFO("blobCenter.x: %f \t\t blobCenter.y: %f", blobCenter.x, blobCenter.y);
			// ROS_INFO("###################### roi.cols: %i \t\t roi.rows: %i", roi.cols, roi.rows);
			// if(roi.cols < 10 || roi.rows < 10) ROS_INFO("###################### roi.cols < 10 || roi.rows < 10");

			cv:Scalar distance = cv::mean(roi);        // compute mean value of the region of interest.
														//    RECALL: the pixels correspond to distance in mm.


//			cout << std::fixed << std::setprecision( 3 );
//			cout << endl;
//			for (int i = 0; i < roi.cols; i++){
//				for(int j = 0; j < roi.rows; j++){
//					cout << roi.at<float>(i, j)/1000.0 << " ";
//				}
//				cout << endl;
//			}

//			cout << endl;
//			for (int i = 0; i < roi.cols; i++){
//				for(int j = 0; j < roi.rows; j++){
//					cout << asdasd(roi.at<float>(i, j)/1000.0);
//				}
//				cout << endl;
//			}
//
//			cout << asd_counter++ << endl;


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
				segmentDepth(depthmat, segmat, blobCenter.x, blobCenter.y, ci, 300);


				// phi is the angular coordinate for the width
				float rho = meanDistance;
				float phi = (0.5 - blobCenter.x / KIN_WIDTH) * WIDTH_FOV;
				float theta = M_PI / 2 - (0.5 - blobCenter.y / KIN_HEIGHT) * HEIGHT_FOV;

				float x = rho * sin(theta) * cos(phi);
				float y = rho * sin(theta) * sin(phi);
				float z = rho * cos(theta);

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

        ////////////////////////////////////////////////


        /* CREATE ROS MESSAGE*/
        robogame_kinectfeatures_extractor::kinect_feat msg;
        msg.header.stamp = ros::Time::now();
        std_msgs::Int32 xposition;

        ROS_INFO_STREAM(isPlayerMissing << "--" << previousDistance << "--" << meanDistance);
        if (isPlayerMissing){
            // Trims values given missing player.
            if(previousDistance <= MIN_DISTANCE){
                ROS_WARN_STREAM("PLAYER TOO CLOSE!");
                msg.ci = previousCI;
                msg.distance = MIN_DISTANCE;
                msg.proximity = (1 - (MIN_DISTANCE/4.500f));
            }else if(previousDistance >= MAX_DISTANCE){
                ROS_WARN_STREAM("PLAYER TOO FAR AWAY!");
                msg.ci = previousDistance;
                msg.distance = MAX_DISTANCE;
                msg.proximity = (1 - (MAX_DISTANCE/4.500f));
            }

            if (blobCenter.x < 256 ){
            	xposition.data = 0;
            }else{
            	xposition.data = 512;
            }

        }else{
            msg.ci = ci;
            msg.distance = meanDistance;
            msg.proximity = (1 - (meanDistance/4.500f)); // Normalize distance (proximity)

            // saves previous distance value.
            previousDistance = meanDistance;
            // saves previous ci value.
            previousCI = ci;
            xposition.data = blobCenter.x;
        }

        //Publish the message
        pub.publish(msg);
        position_pub.publish(xposition);        // publishes blob x coordinate.
        previousBlobCenter = blobCenter;

        segmentedColorFrame.convertTo(segmentedColorFrame, CV_8UC4);
        sensor_msgs::ImagePtr segmentationmsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", segmentedColorFrame).toImageMsg();
        segmentationPub.publish(segmentationmsg);


        //Send a message to rosout with the details.
        ROS_INFO_STREAM("KINECT FEATURES: "
            << " ci="  << msg.ci
            << " distance=" << msg.distance
            << " proximity=" << msg.proximity);
		}

        /* Update/show images */
        cv::imshow("undistorted", undistortedFrame);
        cv::imshow("registered", regframe);;
        cv::imshow("segmentation", segmentedColorFrame);

        int key = cv::waitKey(1);

        /* Evaluate exit condition*/
        protonect_shutdown = protonect_shutdown || (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape

        /*release the frames and loop back*/
        listener.release(frames);
		count++;
        //Wait until it's time for another iteration.
        rate.sleep();
    }

    /* PERFORMING CLEANING OPERATIONS ON EXIT*/
    dev->stop();
    dev->close();
    delete registration;
    std::cout << "Shutting down!" << std::endl;
    return 0;
}
