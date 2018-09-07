#ifndef RANSAC_HPP
#define RANSAC_HPP

/** 
 *  @file    random_sample_consensus.h
 *  @author  Ewerton Lopes
 *  @date    25/07/2018  
 *  @version 1.0 
 *  
 *  @brief detects lines in the Laser data.
 *
 *  @section DESCRIPTION
 *  Uses RANSAC for detecting lines in the LaserScan msg. Such lines are
 *  then assumed to be the playground walls.
 *
 */

// built-in
#include <iostream>
#include <string>
#include <vector>

// ROS core headers
#include <ros/ros.h>

// ROS packages
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h> 
#include <sensor_msgs/LaserScan.h>

// third-part libraries
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <boost/thread/thread.hpp>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>


namespace PlayerTracking{
    class Ransac{
        public:

            /**
            *  @brief Constructor
            */
            Ransac(ros::NodeHandle nh, std::string base_frame);

            /**
            *  @brief converts a LaserScan into PointCloud
            *  @param scanMsg laserScan message
            */
            sensor_msgs::PointCloud laserMsgAsPointCloud(const sensor_msgs::LaserScan::ConstPtr &msg);

            /**
            *  @brief callback for laser scan message
            *  @param scanMsg the laser scan message.
            */
            void laserCallback(const sensor_msgs::LaserScan::ConstPtr &scanMsg);

            /**
            *  @brief run Ransac
            *  @param cloud cloud point to process.
            */
            std::vector<int> runRansac(pcl::PointCloud<pcl::PointXYZ>::Ptr in);

            /**
            *  @brief Setup publishers and subscriber nodes.
            */
            void initRosComunication();


            void pointcloud_to_laserscan(Eigen::MatrixXf points, sensor_msgs::LaserScanPtr output);

        private:

            laser_geometry::LaserProjection projector_;                 ///< object to transform LaserScan data.
            
            ros::NodeHandle nh_;                                        ///< private ROS node handle
            ros::Subscriber laser_scan_subscriber_;                     ///< subscriber object for LaserScan messages
            ros::Publisher obstacles_pub_;
            ros::Publisher inliers_pub_;                               ///< publisher object for LaserScan as PointCloud
            ros::Publisher cloud_pub_;

            tf::TransformListener tf_listener_;                          ///< ROS TF listener.

            std::string base_frame_;

    };
};
#endif