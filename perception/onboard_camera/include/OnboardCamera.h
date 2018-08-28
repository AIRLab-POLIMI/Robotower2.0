/*
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

#ifndef TRACKER_H
#define TRACKER_H

/* ROS related includes */
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

/* OpenCV related includes */
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

/* Local includes */
#include "utils.h"

#define MIN_DISTANCE 0.3
#define MAX_DISTANCE 4.2
#define MIN_BLOB_RADIUS 15

class OnboardCamera{
private:

    Eigen::Matrix<float, 2, 1> x;  // initial state (location and velocity)
    Eigen::Matrix<float, 2, 1> u;  // external motion (NO CONTROL INPUT ASSUMED) 
    Eigen::Matrix<float, 1, 2> H;  // measurement function 6 states - 2 observed (angle and distance)
    Eigen::Matrix<float, 2, 2> I;  // identity matrix
    Eigen::Matrix<float, 1, 1> R;  // measurement uncertainty (2 uncorrelated measures with uncertainty)
    Eigen::Matrix<float, 2, 2> P;  // initial uncertainty
    Eigen::Matrix<float, 2, 2> F;  // Transition Matrix
    Eigen::Matrix<float, 2, 2> Q;  // process noise matrix

    ros::Publisher pub_angle_offset;
    ros::Publisher pub_angle_offset_kalman;
    image_transport::Publisher pub_result_image;

    std::string cam_ns;
    std::string color_frame_name;
    
    /* HSV space variables for blob detection */
    int hMin;
    int sMin;
    int vMin;
    int hMax;
    int sMax;
    int vMax;

    float phi_target;

    int cam_width;
    int cam_height;
    float width_fov;
    float height_fov;
    float seg_threshold;

    bool is_exit;
    bool is_shutdown;
    bool show_frame;

    cv::Point2f blob_center;    // variable for blob center tracking at time t.
    cv::Point2f pr_blob_center; // variable for blob center tracking at time t-1.
    cv::Mat seg_color_frame;    // the segmented color frame.
    cv::Mat seg_target;
    cv::Mat rgbmat;

    boost::circular_buffer<float> pts; // The blob location history

    tf::TransformListener* tfListener;  // Global tf listener pointer


public:

    ros::NodeHandle nh;

    // Image transport handle
    image_transport::ImageTransport it;

    // Create a subscriber.
    image_transport::Subscriber sub;

    ros::Subscriber sub_servo_angle;
    ros::Publisher pub_servo_angle;

    double current_servo_angle;
    int angle_when_lost_player;
    double vel_angle; /*Deprecated*/

    std::string pub_topic_centres;
    std::string pub_topic_ubd;
    std::string pub_topic_result_image;
    std::string topic_color_image;

    int queue_size;

    tf::Vector3 cam_pos;
    tf::Vector3 target_pos;

    OnboardCamera();
    
    void kalmanPredict();
    void kalmanUpdate(float angle);
    
    int findColorBlob(cv::Mat& srcFrame,  cv::Point2f &blob_center);
    void resize(cv::Mat& sourceFrame,cv::Mat& resultingFrame,int width=512, int height=-1);
    
    
    void servoAngleCallback(const std_msgs::Int16Ptr &msg);
    void velCallback(const geometry_msgs::TwistPtr &msg);
    void imageCallback(const sensor_msgs::ImageConstPtr &image);
    // Connection callback that unsubscribes from the tracker if no one is subscribed.
    void connectCallback(image_transport::SubscriberFilter &sub_col,
                         image_transport::SubscriberFilter &sub_dep,
                         image_transport::ImageTransport &it);
    
    void publishCameraTF();

};

#endif