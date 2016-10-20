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

/* HSV space variables for blob detection */
int hMin = 127;
int sMin = 99;
int vMin = 39;
int hMax = 165;
int sMax = 256;
int vMax = 256;
/* ... */

cv::Point2f mainCenter;         // variable for blob center tracking
bool missedPlayer;              // a flag for the player presence.

const int BUFFER = 32;          // the object trail size
boost::circular_buffer<cv::Point2f> pts(BUFFER);    // The blob location history

cv::Mat segmentedColorFrame;    // the segmented color frame.
cv::Mat segmentedTarget;

using namespace std;
using namespace cv;

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
	//image_transport::Publisher imgPub = it.advertise("camera/image/depth", 1);

	//Create a publisher object.
    ros::Publisher pub = nh.advertise<robogame_kinectfeatures_extractor::kinect_feat>("kinect_features",1000);

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
    cv::Mat rgbmat, depthmat, depthmatUndistorted, irmat, rgbd, rgbd2, depthAndRgb, regframe;
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
        cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
        cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);
        cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(regframe);

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
        cv::Mat undistortedFrame = cv::Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data);
        cv::Mat segmat = Mat::zeros(undistortedFrame.size(), undistortedFrame.type());
        segmentedColorFrame = Mat::zeros(undistortedFrame.size(), CV_8UC3);
        segmentedTarget = Mat::zeros(undistortedFrame.size(), CV_8UC3);

        /* THIS LOOP COMPUTES A REGION OF INTEREST (A CIRCLE) BASED ON THE mainCenter VARIABLE COMPUTED BY
            THE trackUser METHOD. THE IDEA IS THEN TO ASSESS THE MEAN DISTANCE (PIXEL VALUES)
            DEFINED IN THIS AREA AND THUS OBTAIN THE DISTANCE FEATURE. THE SAME LOOP ALSO CALL THE
            segmentDepth METHOD IN ORDER TO OBTAIN THE CONTRACTION INDEX FEATURE.*/
        if ((mainCenter.x != -1000) && (mainCenter.x != 0)){
            int radius = 5;

            //get the Rect containing the circle:
            cv::Rect r(mainCenter.x-radius, mainCenter.y-radius, radius*2,radius*2);
            // obtain the image ROI:
            cv::Mat roi(undistortedFrame, r);

            // make a black mask, same size:
            cv::Mat maskROI(roi.size(), roi.type(), cv::Scalar::all(0));

            // with a white, filled circle in it:
            cv::circle(maskROI, cv::Point(radius,radius), radius, cv::Scalar::all(255), -1);

            // combine roi & mask:
            cv::Mat roiArea = roi & maskROI;
            // -------

            cv:Scalar m = cv::mean(roi);        // compute mean value of the region of interest.
                                            //    RECALL: the pixels correspond to distance in mm.
            meanDistance = m[0] / 1000.0f;  // compute distance (in meters)

            /* perform segmentation in order to get the contraction index featue.
                The result will be saved in ci variable*/
            segmentDepth(undistortedFrame, segmat, mainCenter.x, mainCenter.y, ci, 300);
        }

        /* A LOOP TO PRINT THE DISTANCE AND HISTORY TRACE IN THE DEPTH FRAME. IT JUST SERVES AS A
         VISUAL AID OF WHAT IS GOING ON.*/
    	for (int i=1; i < (pts.size()-1); i++){
    		// if either of the tracked points are None, ignore
    		// them

    		cv::Point2f ptback = pts[i - 1];
    		cv::Point2f pt = pts[i];
    		if ((ptback.x == -1000) or (pt.x == -1000)){
                continue;
    		}

    		// otherwise, compute the thickness of the line and
    		// draw the connecting lines
    		int thickness = int(sqrt(BUFFER / float(i + 1)) * 2.5);
    		line(undistortedFrame, pts[i - 1], pts[i], cv::Scalar(0, 0, 255), thickness);
            // Write distance
            cv::putText(undistortedFrame,
            std::to_string(meanDistance),
            cv::Point((512/2)-60,85), // Coordinates
            cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
            1.0, // Scale. 2.0 = 2x bigger
            cv::Scalar(255,255,255), // Color
            1 // Thickness
            ); // Anti-alias
    	}

        /* CREATE ROS MESSAGE*/
        robogame_kinectfeatures_extractor::kinect_feat msg;

        if (missedPlayer){
            ROS_WARN_STREAM("PLAYER IS MISSING!");
            msg.header.stamp = ros::Time::now();
            msg.ci = -1;
            msg.distance = -1;
            msg.proximity = -1;
            //msg.crosstrack = -1;
        }else{
            msg.header.stamp = ros::Time::now();
            msg.ci = ci;
            msg.distance = meanDistance;
            msg.proximity = (1 - (meanDistance/4.500f)); // Normalize distance (proximity)
            //msg.crosstrack = (registered.width/2) - mainCenter.x;
        }

        //Publish the message
        pub.publish(msg);

        //Send a message to rosout with the details.
        ROS_INFO_STREAM("KINECT FEATURES: "
            << " ci="  << msg.ci
            << " distance=" << msg.distance
            << " proximity=" << msg.proximity);

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
