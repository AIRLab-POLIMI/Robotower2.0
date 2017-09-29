/* main.cpp -- This file is part of the robogame_kinect_bridge ROS node created for
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
#include <robogame_kinect_bridge/kinect_feat.h>
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
// #include "utils.h"
#include "common.h"
/* ... */


using namespace std;
using namespace cv;

enum Processor { cl, gl, cpu };         // libfreenect2 enum.

bool protonect_shutdown = false;        // Whether the running application should shut down.

/* Interruption handler function. In this case, set the variable that controls the
 frame aquisition, breaking the loop, cleaning variables and exit the program elegantly*/
void sigint_handler(int s){
    protonect_shutdown = true;
}

/* printMatImage -- a function to print a cv::Mat to console. Used
mostly for debugging purposes*/
void printMatImage(cv::Mat frame){
    printf("\n --> Matrix type is CV_32F \n");

    for( size_t i = 0; i < frame.rows; i++ ) {
        for( size_t j = 0; j < frame.cols; j++ ) {
   	    printf( " %.3f  ", frame.at<float>(i,j) );
 	}
	printf("\n");
    }
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
    ros::Publisher pub = nh.advertise<robogame_kinect_bridge::kinect_feat>("kinect_features",1000);
    ros::Publisher position_pub = nh.advertise<std_msgs::Int32>("robogame/player_x_position",1000);

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
	//cv::VideoWriter videoWriter("/home/airlab/catkin_ws/src/robogame_kinect_bridge/videos/out.avi",CV_FOURCC('M', 'J', 'P', 'G'),30.0,cv::Size(undistorted.height,undistorted.width),true);


    // ROS loop at 2Hz until the node is shut down.
    ros::Rate rate(40);

    //! [loop start] -- DATA AQUISITION
    while(!protonect_shutdown && ros::ok())
    {
        /* Kinect data aquisition */
        listener.waitForNewFrame(frames);
        libfreenect2::Frame *colorFrame = frames[libfreenect2::Frame::Color];
        // libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

        // cout << "1: rgb->width: " << rgb->width << endl;
        //Apply and read the registration video stream
        // registration->apply(rgb,depth,&undistorted,&registered);
        /* ---- */

        cout << "2: colorFrame: " << colorFrame << endl;
        /* Save the aquired data to the cv::mat containers. */
        // cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
        // cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);
        // cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(regframe);


        cv::Mat color = cv::Mat(colorFrame->height, colorFrame->width, CV_8UC4, colorFrame->data);
   
        cout << "colorFrame->format" << colorFrame->format << endl;

        cv::Mat image;
        cv::Mat tmp;
        cv::flip(color, tmp, 1);

        if(colorFrame->format == libfreenect2::Frame::BGRX) {
            cv::cvtColor(tmp, image, CV_BGRA2BGR);
        } else {
            cv::cvtColor(tmp, image, CV_RGBA2BGR);
        }

        cout << image << endl;

        // printMatImage(rgbmat);


        cout << "2.5: " << endl;
		/*converting the rgb into grayscale.. Why? compression purposes*/
		//cv::Mat im_gray;
		//cvtColor(rgbmat,im_gray,CV_RGB2GRAY);
		//resize(im_gray, im_gray, cv::Size(512,424), 2.28, 2.28, cv::INTER_LANCZOS4);
        // resize(rgbmat, rgbmat, cv::Size(512,424));

        cout << "3: " << endl;
		/* Publishes the image. NOTE:: Here, the rgb image  is in fact a bgr image (bgra8, i.e., CV8U4).
        see http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages */
		//sensor_msgs::ImagePtr imgmsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", im_gray).toImageMsg();
		// sensor_msgs::ImagePtr imgmsg = cv_bridge::CvImage(std_msgs::Header(), "bgra8", rgbmat).toImageMsg();
		// imgPub.publish(imgmsg);
		//imgPub.publish(depthmsg);


        /* Supporting frames*/
        // cv::Mat undistortedFrame = cv::Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data);

        cout << "4: " << endl;
        //sensor_msgs::ImagePtr segmentationmsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", segmentedColorFrame).toImageMsg();
        //segmentationPub.publish(segmentationmsg);


        /* Update/show images */

        // cv::imshow("rgb", rgbmat);
        // cv::imshow("depthmat", depthmat);
        // cv::imshow("undistorted", undistortedFrame);
        // cv::imshow("registered", regframe);

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
