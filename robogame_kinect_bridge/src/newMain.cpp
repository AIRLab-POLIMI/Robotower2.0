/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author: Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <chrono>
#include <sys/stat.h>

#if defined(__linux__)
#include <sys/prctl.h>
#elif defined(__APPLE__)
#include <pthread.h>
#endif

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>

#include <tf/transform_broadcaster.h>

#include <compressed_depth_image_transport/compression_common.h>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/config.h>
#include <libfreenect2/registration.h>

/*
#include <kinect2_bridge/kinect2_definitions.h>
#include <kinect2_registration/kinect2_registration.h>
#include <kinect2_registration/kinect2_console.h>
*/


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>





using namespace std;

enum Processor { cl, gl, cpu };


/* Given a source depth frame applies color map to depth pixels  */
void getDepthColorMapped(const cv::Mat &source, cv::Mat &destination){
	unsigned short min = 500; 		// minimum kinect depth value in mm.
	unsigned short max = 4500;		// maximum kinect depth value in mm.

	cv::Mat img0 = cv::Mat::zeros(source.size().height, source.size().width, CV_8UC1);
	cv::Mat img1;
	double scale = 255.0 / (max - min);
	source.convertTo(img0, CV_8UC1, scale);
	applyColorMap(img0, destination, cv::COLORMAP_JET);
}


void createImage(const cv::Mat &image, const std_msgs::Header &header, sensor_msgs::Image &msgImage) {
    size_t step, size;
    step = image.cols * image.elemSize();
    size = image.rows * step;

    if ((msgImage.encoding != sensor_msgs::image_encodings::BGR8) && (msgImage.encoding != sensor_msgs::image_encodings::BGRA8))
    	msgImage.encoding = sensor_msgs::image_encodings::BGR8;
    
    msgImage.header = header;
    msgImage.height = image.rows;
    msgImage.width = image.cols;
    msgImage.is_bigendian = false;
    msgImage.step = step;
    msgImage.data.resize(size);
    memcpy(msgImage.data.data(), image.data, size);
}

int main(int argc, char** argv)
{
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "publish_kinect");
    ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);
	image_transport::Publisher imgPub = it.advertise("camera/image", 1);
	image_transport::Publisher colorDepthPub = it.advertise("camera/undistorted/depth_color_map", 1);
	image_transport::Publisher depthPub = it.advertise("camera/undistorted/depth", 1);
	image_transport::Publisher registeredPub = it.advertise("camera/registered", 1);

	//Create a publisher object.
 //   ros::Publisher pub = nh.advertise<robogame_kinect_bridge::kinect_feat>("kinect_features",1000);
//    ros::Publisher position_pub = nh.advertise<std_msgs::Int32>("robogame/player_x_position",1000);

    ROS_INFO("Connecting to sensor...");

    //! libfreenect2 [context]
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = nullptr;
    libfreenect2::PacketPipeline *pipeline = nullptr;
    //! [context]

    //! [discovery] of sensor --  Kinect, in this case.
    if(freenect2.enumerateDevices() == 0){
    	ROS_FATAL("no depth camera device connected!");
        return -1;
    }

    // getting the sensor serial.
    string serial = freenect2.getDefaultDeviceSerialNumber();

    ROS_INFO_STREAM("SERIAL: " << serial);
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
                ROS_ERROR("OpenGL pipeline is not supported!");
        #endif
            } else if (depthProcessor == Processor::cl) {
        #ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
                if(!pipeline)
                    pipeline = new libfreenect2::OpenCLPacketPipeline();
        #else
                ROS_ERROR("OpenCL pipeline is not supported!");
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
    	ROS_FATAL("failure opening device!");
        return -1;
    }
    /* ---- */

  //  signal(SIGINT, sigint_handler);             // Set interruption handler
    bool protonect_shutdown = false;                 // Define value of the loop control variable

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
    ROS_INFO_STREAM("device serial: " << dev->getSerialNumber());
    ROS_INFO_STREAM("device firmware: " << dev->getFirmwareVersion());
    //! [start]

    //! [registration setup]
    libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    libfreenect2::Frame undistorted(512, 424, 4);
    libfreenect2::Frame registered(512, 424, 4);
    libfreenect2::Frame depth2rgb(1920, 1080 + 2, 4);
    // check here (https://github.com/OpenKinect/libfreenect2/issues/337) and
    // here (https://github.com/OpenKinect/libfreenect2/issues/464) why depth2rgb image should be bigger
    //! [registration setup]

    int count = 0;

    // ROS loop at 2Hz until the node is shut down.
    ros::Rate rate(40);

    //! [loop start] -- DATA AQUISITION
    while(!protonect_shutdown && ros::ok())
    {
        /* Kinect data aquisition */
        listener.waitForNewFrame(frames);
        libfreenect2::Frame *colorFrame = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *depthFrame = frames[libfreenect2::Frame::Depth];

		//Apply and read the registration video stream
		registration->apply(colorFrame,depthFrame,&undistorted,&registered);

        //cout << "2: colorFrame: " << colorFrame << endl;

        cv::Mat color = cv::Mat(colorFrame->height, colorFrame->width, CV_8UC4, colorFrame->data);
        cv::Mat undistortedFrame = cv::Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data);
        cv::Mat regframe = cv::Mat(registered.height, registered.width, CV_8UC4, registered.data);


        /* FLIPPING IMAGE FRAME*/
        cv::Mat image;
        cv::Mat tmp;
        cv::flip(color, tmp, 1);
        if(colorFrame->format == libfreenect2::Frame::BGRX) {
            cv::cvtColor(tmp, image, CV_BGRA2BGR);
        } else {
            cv::cvtColor(tmp, image, CV_RGBA2BGR);
        }

		/* PUBLISHING RGB IMAGE*/
        sensor_msgs::ImagePtr imageMsg = sensor_msgs::ImagePtr(new sensor_msgs::Image);
		std_msgs::Header header;
		header.stamp = ros::Time::now();
        createImage(image, header, *imageMsg);
		imgPub.publish(imageMsg);
		/**********************/
		
		/* FLIPPING UNDISTORTED DEPTH FRAME*/
		cv::Mat und_depth_image;
		cv::flip(undistortedFrame, und_depth_image, 1);

		/* PUBLISHING UNDISTORTED DEPTH IMAGE*/
		sensor_msgs::ImagePtr undistortedDepthMsg = sensor_msgs::ImagePtr(new sensor_msgs::Image);
		std_msgs::Header undistorted_header;
		header.stamp = ros::Time::now();
		createImage(und_depth_image, undistorted_header, *undistortedDepthMsg);
		depthPub.publish(undistortedDepthMsg);
		/**********************/

		/* FLIPPING UNDISTORTED COLORMAP DEPTH FRAME*/

		cv::Mat colorMapped;
		getDepthColorMapped(undistortedFrame, colorMapped);

		cv::Mat und_colormap_depth_image;
		cv::flip(colorMapped, und_colormap_depth_image, 1);

		/* PUBLISHING UNDISTORTED COLORMAP DEPTH IMAGE*/
		sensor_msgs::ImagePtr colorMapMsg = sensor_msgs::ImagePtr(new sensor_msgs::Image);
		std_msgs::Header color_mapped_header;
		header.stamp = ros::Time::now();
		createImage(und_colormap_depth_image, color_mapped_header, *colorMapMsg);
		colorDepthPub.publish(colorMapMsg);
		/**********************/

		/* FLIPPING REGISTERED FRAME*/
		cv::Mat redistered_image;
		cv::flip(regframe, redistered_image, 1);

		/* PUBLISHING REGISTERED IMAGE*/
		sensor_msgs::ImagePtr registeredMsg = sensor_msgs::ImagePtr(new sensor_msgs::Image);
		registeredMsg->encoding = sensor_msgs::image_encodings::BGRA8;
		std_msgs::Header registered_header;
		header.stamp = ros::Time::now();
		createImage(redistered_image, registered_header, *registeredMsg);
		registeredPub.publish(registeredMsg);
		/**********************/


        /* SHOWING UP FRAMES */
//        cv::imshow("colorMapped", colorMapped);
//        cv::imshow("regFrame", redistered_image);
        //cv::imshow("undistorted", undistortedFrame);

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
    ROS_INFO_STREAM("Shutting down!");
    return 0;
}






