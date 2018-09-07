#include <iostream>
#include <ros/ros.h>
// PCL specific includes
#include <boost/lexical_cast.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

// ros::Publisher pub;
std::vector<ros::Publisher> pub_vec;
sensor_msgs::PointCloud2::Ptr downsampled, output;
// pcl::PointCloud<pcl::PointXYZ>::Ptr output_p, converted;

/*
Example of pcl usage where we try to use clustering to detect objects
in the scene.

Original code taken from:
http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php#cluster-extraction


Usage:
    first run: rosrun pcl_ros pcd_to_pointcloud <file.pcd> [ <interval> ]
    then run: rosrun my_pcl_tutorial cluster_extraction
    and look at the visualization through rviz with:
    rosrun rviz rviz

    Set /base_link as the Fixed Frame, check out the published PointCloud2
topics

*/

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input) {
  pcl::PCLPointCloud2 to_convert;
  
  // Change from type sensor_msgs::PointCloud2 to pcl::PointXYZ
  pcl_conversions::toPCL(*input, to_convert);

  pcl::PointCloud<pcl::PointXYZ>::Ptr converted(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(to_convert,*converted);

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(converted);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.05); // 5cm
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(converted);
  ec.extract(cluster_indices);

  ros::NodeHandle nh;

  // Create a publisher for each cluster
  for (int i = 0; i < cluster_indices.size(); ++i) {
    std::string topicName = "/pcl_tut/cluster" + boost::lexical_cast<std::string>(i);
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>(topicName, 1);
    pub_vec.push_back(pub);
  }

  int j = 0;

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
  it != cluster_indices.end(); ++it) {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin();
    pit != it->indices.end(); pit++)
      cloud_cluster->points.push_back(converted->points[*pit]); //*
    
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    // std::cout << "PointCloud representing the Cluster: " <<
    // cloud_cluster->points.size () << " data points." << std::endl;

    // Convert the pointcloud to be used in ROS
    sensor_msgs::PointCloud2::Ptr output(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*cloud_cluster, *output);
    output->header.frame_id = input->header.frame_id;

    // Publish the data
    pub_vec[j].publish(output);
    ++j;
  }
}

int main(int argc, char **argv) {

  // Initialize ROS
  ros::init(argc, argv, "cluster_extraction");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("/cloud_pcd", 1, cloud_cb);

  ros::spin();
}