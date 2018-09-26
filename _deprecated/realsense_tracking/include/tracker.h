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

/* OpenCV related includes */
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <eigen3/Eigen/Dense>

/* Local includes */
#include "utils.h"
//#include "common.h"

#define MIN_DISTANCE 0.3
#define MAX_DISTANCE 4.2
#define MIN_BLOB_RADIUS 15
#define BUFFER_DIST_SIZE 5

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;

class PlayerTracker{
private:

    Eigen::Matrix<float, 6, 1> x;  // initial state (location and velocity)
    Eigen::Matrix<float, 6, 1> u;  // external motion (NO CONTROL INPUT ASSUMED) 
    Eigen::Matrix<float, 2, 6> H;  // measurement function 6 states - 2 observed (angle and distance)
    Eigen::Matrix<float, 6, 6> I;  // identity matrix
    Eigen::Matrix<float, 2, 2> R;  // measurement uncertainty (2 uncorrelated measures with uncertainty)
    Eigen::Matrix<float, 6, 6> P;  // initial uncertainty
    Eigen::Matrix<float, 6, 6> F;  // Transition Matrix
    Eigen::Matrix<float, 6, 6> Q;  // process noise matrix

    ros::Publisher pub;
    ros::Publisher pub_kalman;
    ros::Publisher pub_player_pos_wrt_map;
    ros::Publisher pub_player_pos_wrt_robot;

    image_transport::Publisher pub_result_image;

    std::string cam_ns;
    std::string color_frame_name;
    std::string depth_frame_name;

    int cam_width;
    int cam_height;
    float width_fov;
    float height_fov;
    float seg_threshold;

    /* HSV space variables for blob detection */
    int hMin;
    int sMin;
    int vMin;
    int hMax;
    int sMax;
    int vMax;

    cv::Point2f blob_center;    // variable for blob center tracking at time t.
    cv::Point2f pr_blob_center; // variable for blob center tracking at time t-1.

    float mean_distance;        // distance feature
    float pr_mean_distance;     // variable for distance at time t-1.
    float ci ;                  // contraction index
    float pr_ci;                // variable for ci at time t-1.

    bool is_player_missing;     // a flag for the player presence.
    bool is_exit;
    bool is_shutdown;
    bool show_frame;

    boost::circular_buffer<float> distance_buffer;  // a buffer to low-pass the kinect distance
    tf::TransformListener* tfListener;  // Global tf listener pointer

    cv::Mat seg_color_frame;    // the segmented color frame.
    cv::Mat seg_target;
    cv::Mat rgbmat;

    Eigen::Matrix<float, 3, 1> getSphericalCoordinate(float rho, float theta, float phi);


public:

    ros::NodeHandle nh;

    std::string pub_topic_centres;
    std::string pub_topic_ubd;
    std::string pub_topic_result_image;
    std::string pub_topic_detected_persons;
    std::string topic_color_image;
    std::string topic_depth_image;

    int queue_size;

    PlayerTracker();
    int findColorBlob(cv::Mat& srcFrame,  cv::Point2f &blob_center);
    float segmentDepth(cv::Mat& inputFrame, cv::Mat& resultingFrame, int sX, int sY, float& threshold);
    void resize(cv::Mat& sourceFrame,cv::Mat& resultingFrame,int width=512, int height=-1);
    void kalmanUpdate(float angle, float distance);
    void kalmanPredict();
    void callback(const sensor_msgs::ImageConstPtr &depth, const sensor_msgs::ImageConstPtr &image);
    void publishCameraTF();
    void publishKalman();
    // Connection callback that unsubscribes from the tracker if no one is subscribed.
    void connectCallback(image_transport::SubscriberFilter &sub_col, image_transport::SubscriberFilter &sub_dep, image_transport::ImageTransport &it);

};

#endif