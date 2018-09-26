#ifndef LASER_CLUSTER_EXTRACTOR_HPP
#define LASER_CLUSTER_EXTRACTOR_HPP

/*********************************************************************
*  @author Ewerton Lopes (ewerton.lopes@polimi.it)
*
*  @date Jul 10th 2018
*
*  @version 1.0
*
*  @description This software listen to 2D laser scan topics and
*  publishes clusters of data found on them.
*
*  @license Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

// C++ built-in headers
#include <string>
#include <set>

// ROS built-in headers
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>

// Local headers
#include <player_tracker/laser_processor.h>

// Custom message headers
#include <player_tracker/Cluster.h>
#include <player_tracker/ClusterArray.h>


/**
* @brief Extract and publish laser clusters.
*/
class LaserClusterExtractor{

private:

    std::string scan_topic_;
    std::string fixed_frame_;
    
    bool use_scan_header_stamp_for_tfs_;
    double cluster_dist_euclid_;
    double max_detect_distance_;
    int min_points_per_cluster_;

    tf::TransformListener tfl_;

    ros::NodeHandle nh_;
    ros::Subscriber laser_scan_subscriber_;
    ros::Publisher detected_laser_clusters_pub_;
    
public:
    LaserClusterExtractor();
    
    /**
     * @brief Called every time a laser scan is published.
     */
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
};

#endif