#ifndef PLAYER_TRACKER_SPATIAL_TEMPORAL_H_
#define PLAYER_TRACKER_SPATIAL_TEMPORAL_H_

/** 
 *  @file    spatial_temporal.h
 *  @author  Ewerton Lopes
 *  @author  Luca Morreale
 *  @date    15/02/2018  
 *  @version 1.0 
 *  
 *  @brief extracts spatial-temporal features from Laser data.
 *
 *  @section DESCRIPTION
 *  This program reuse some spatial-temporal modeling ideas from 
 *  Shen, Xiaotong, Seong-Woo Kim, and Marcelo H. Ang. "Spatio-temporal 
 *  motion features for laser-based moving objects detection and tracking.
 *  In IEEE/RSJ Internation Conference on Intelligent Robots and Systems (IROS 2014).
 *  September 14-18, Chicago, IL, USA.
 *
 */


// Linear algebra
#include <eigen3/Eigen/Dense>

// Optmization
#include <omp.h>

// ROS core headers
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <laser_assembler/AssembleScans.h>

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

// Custom ROS messages
#include <player_tracker/LegArray.h>
#include <player_tracker/TrackVariance.h>

// ROS messages
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Header.h>

// C++ headers
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>    // std::sort
#include <iterator>
#include <utility>      // std::pair
#include <cmath>       /* acos */

#define PI 3.14159265

typedef std::vector<sensor_msgs::PointCloud2> Cloud2List;
typedef std::vector<sensor_msgs::PointCloud2> PointCloud2List;
typedef std::vector<sensor_msgs::PointCloud> PointCloudList;

#define OMP_THREADS 8

namespace spatial_temporal{

typedef struct MinimalPointCloud{
    float variance;
    Eigen::VectorXd centroid;
    sensor_msgs::PointCloud points;
} MinimalPointCloud;

/**
*  @brief a comparison fuction used for the set std template class.
*/
struct{
    bool operator()(const geometry_msgs::Point32 &a, const geometry_msgs::Point32 &b){
        return ( a.x < b.x && a.y < b.y && a.z < b.z );
    }
}compPoints;

struct{
  bool operator() (std::pair <int,float> i , std::pair <int,float> j) {
      return i.second < j.second; }
} compPair;

/**
*  @class Extractor
*  @brief Extracts spatial-temporal features from LaserScan msgs.
*/
class Extractor {
public:
    /**
    *  @brief Constructor
    *  @param nh A nodehandle
    *  @param scanTopic the scan topic we would like to use
    *  @param windowDuration the slide windows duration in seconds
    *  @param logFilename the name of the file to save log data
    */
    Extractor(ros::NodeHandle nh, std::string scanTopic, float windowDuration, std::string logFilename);

    /**
    *  @brief Constructor
    *  @param nh A nodehandle
    *  @param scanTopic the scan topic we would like to use
    *  @param windowDuration the slide windows duration in seconds
    */
    Extractor(ros::NodeHandle nh, std::string scanTopic, float windowDuration);

    /**
    *  @brief Destructor
    */
    ~Extractor();
    
    /**
    *  @brief Setup publishers and subscriber nodes.
    */
    void initRosComunication();

    /**
    *  @brief Publish a CloudPoint composed by a set of other PointClouds.
    *  @param from the start time of composing clouds.
    *  @param to the end time of composing clouds.
    */    
    void publishAssembledCloud(ros::Time from, ros::Time to);

    /**
    *  @brief Publish a CloudPoint composed by a set of other PointClouds.
    *  @param from the start time of composing clouds.
    *  @param to the end time of composing clouds.
    */  
    void publishAssembledCloud(sensor_msgs::PointCloud cloud);

    /**
    *  @brief Publish an arrow to represent the direction of eigenvector.
    *  @param header the message ROS header.
    *  @param eigenvect eigenvector
    *  @param marker_id marker id
    *  @param offset an offset to the tail of the arrow.
    */  
    void publishEigenvectorMarker(std_msgs::Header header, Eigen::VectorXd eigenvect,
                                  int marker_id, geometry_msgs::Point offset, float marker_lifetime);

    /**
    *  @brief set point cloud z (temporal) dimension.
    *  @param cloud the Pointcloud.
    *  @param time the value to set.
    */  
    void setTemporalDimension(sensor_msgs::PointCloud &cloud, double time);

    /**
    *  @brief Save string to logfile.
    *  @param text the string data
    */  
    void saveToLogFile(std::string text);

    /**
    *  @brief Save PointCloud data to file
    *  @param cloud the PointCloud
    */  
    void saveCloudDataToLog(sensor_msgs::PointCloud &cloud);

    /**
    *  @brief Save projections to logfile.
    */  
    void saveProjectionToFile();

    /**
    *  @brief Compute spatial-temporal data from current window.
    */  
    void computeSpatialTemporalFeatures();

    /**
    *  @brief Clear PointCloud vector.
    */  
    void clearWindowCloudList();

    /**
    *  @brief Append cloud to PointCloud list.
    *  @param cloud the cloud to append
    */  
    void appendWindowCloudList(sensor_msgs::PointCloud &cloud);

    /**
    *  @brief Complete Me
    *  @param cloud the cloud to append
    */  
    void publishVariance(MinimalPointCloud &minimalCloud);

    /**
    *  @brief Complete Me
    *  @param cloud the cloud to append
    */  
    void clearCurrentClusterList();

     /**
    *  @brief Complete Me
    *  @param cloud the cloud to append
    */  
    void publishClusterTextMarker(int marker_id, geometry_msgs::Point position);

    /**
    *  @brief Computes Jaccard Similarity between two CloudPoints.
    *  @param cloud1 1st cloud point.
    *  @param cloud2 2nd cloud point.
    */  
    float computeJaccardSimilarity(sensor_msgs::PointCloud &cloudIn1, sensor_msgs::PointCloud &cloudIn2);

    /**
    *  @brief Compute angle between eigenvector and normal.
    *  @param eigenvect the eigenvector.
    */
    float getAngleWithNormal(Eigen::VectorXd &eigenvect);

    /**
    *  @brief Assemble PointClouds.
    *  @param from the start time of composing clouds.
    *  @param to the end time of composing clouds.
    *  @param cloud the resulting cloud. Results in an empty cloud in case
    *               no composing clouds are available from the specified time interval.
    *  @param status return 1 if suceeded. 0 otherwise.
    */  
    int assembledCloud(ros::Time from, ros::Time to, sensor_msgs::PointCloud &cloud);

    /**
    *  @brief COMPLETE ME.
    *  @param cluster
    */  
    std::vector<std::pair <int,float>> getSimilarity(sensor_msgs::PointCloud2 &cluster);

    /**
    *  @brief Compute clusters in Assembled PointCloud from current windows.
    */  
    Cloud2List getClustersInWindow();
    
    /**
    *  @brief Compute clusters from PointCloud.
    *  @param input the point cloud to compute from.
    */  
    Cloud2List getClusters(sensor_msgs::PointCloud2 &input);
    
    /**
    *  @brief Publish a CloudPoint composed by a set of other PointClouds.
    *  @param from the start time of composing clouds.
    *  @param to the end time of composing clouds.
    */  
    PointCloudList& getWindowCloudList();
    
    /**
    *  @brief Publish a CloudPoint composed by a set of other PointClouds.
    *  @param from the start time of composing clouds.
    *  @param to the end time of composing clouds.
    */  
    geometry_msgs::Point computeClusterMean(sensor_msgs::PointCloud2 &input);
    
    /**
    *  @brief converts a LaserScan into PointCloud
    *  @param scanMsg laserScan message
    */
    sensor_msgs::PointCloud laserMsgAsPointCloud(const sensor_msgs::LaserScan::ConstPtr &scanMsg);

    /**
    *  @brief project points onto eigenvector.
    *  @param dataset the data to convert.
    *  @param eigenvect the eigenvecto to project onto.
    */
    Eigen::MatrixXd projectToEigenvector(Eigen::MatrixXd &dataset, Eigen::VectorXd &eigenvect);
    
    /**
    *  @brief Publish a CloudPoint composed by a set of other PointClouds.
    *  @param from the start time of composing clouds.
    *  @param to the end time of composing clouds.
    */  
    Eigen::MatrixXd cloudPointAsMatrix(sensor_msgs::PointCloud2 &input);
    
    /**
    *  @brief Publish a CloudPoint composed by a set of other PointClouds.
    *  @param from the start time of composing clouds.
    *  @param to the end time of composing clouds.
    */  
    Eigen::MatrixXd cloudPointAsMatrix(sensor_msgs::PointCloud &cloud);
    
    /**
    *  @brief Publish a CloudPoint composed by a set of other PointClouds.
    *  @param from the start time of composing clouds.
    *  @param to the end time of composing clouds.
    */  
    Eigen::MatrixXd computeClusterVariance(Eigen::MatrixXd &input);
    
    /**
    *  @brief Publish a CloudPoint composed by a set of other PointClouds.
    *  @param from the start time of composing clouds.
    *  @param to the end time of composing clouds.
    */  
    Eigen::MatrixXd computeClusterVariance(sensor_msgs::PointCloud2 &input);

    /**
    *  @brief COMPLETE ME
    *  @param from the start time of composing clouds.
    *  @param to the end time of composing clouds.
    */  
    Eigen::MatrixXd computeMinEigenvectors(Cloud2List &clusters);

    /**
    *  @brief Publish a CloudPoint composed by a set of other PointClouds.
    *  @param from the start time of composing clouds.
    *  @param to the end time of composing clouds.
    */  
    Eigen::MatrixXd::Index findSteepestEigenvector(Eigen::MatrixXd eigenvectors);

    /**
    *  @brief Publish a CloudPoint composed by a set of other PointClouds.
    *  @param from the start time of composing clouds.
    *  @param to the end time of composing clouds.
    */  
    Eigen::VectorXd computeSteepestDirection(Cloud2List &clusters);
    
    /**
    *  @brief Publish a CloudPoint composed by a set of other PointClouds.
    *  @param from the start time of composing clouds.
    *  @param to the end time of composing clouds.
    */  
    Eigen::VectorXd getClusterSmallestEigenvector(Eigen::MatrixXd cov);
    
    /**
    *  @brief Compute the vector with smallest eigenvalue from PointCloud.
    *  @param cluster the PointCloud cluster to compute from.
    */  
    Eigen::VectorXd getClusterSmallestEigenvector(sensor_msgs::PointCloud2 &cluster);

private:
    std::string scanTopic_;                   ///< holds scan topic
    std::string baseFrame_;                   ///< robot base frame
    std::ofstream logFile_;                   ///< log file stream
    bool onRunningWindow_;                    ///< a flag to tell whether we have a current windows running.
    int window_counter_;                     ///< tracks how many times a windows is processed
    int win_stamp_counter_;

    ros::NodeHandle nh_;                      ///< private ROS node handle
    ros::Subscriber laserScanSubscriber_;     ///< subscriber object for LaserScan messages
    ros::Subscriber legArraySubscriber_;      ///< subscriber object for LegArray messages
    ros::Publisher laserToCloudPublisher_;    ///< publisher object for LaserScan as PointCloud
    ros::Publisher assembledCloudPublisher_;  ///< publisher object for assembled PointClouds
    ros::Publisher cloudClusterPublisher_;    ///< publisher object for clusters found in a merged PointCloud.
    ros::Publisher cloudClusterTextPublisher_;///< publisher object for text clusters found in a merged PointCloud.
    ros::Publisher legCloudPublisher_;        ///< publisher object for leg clouds
    ros::Publisher markerPublisher_;          ///< publisher object for markers.
    ros::Duration windowDuration_;            ///< slide window duration.
    ros::Time startTime_;                    ///< windows start time.
    ros::ServiceClient assembleScanServiceClient_;              ///< assemble scan service client.

    tf::TransformListener tfListener_;                          ///< ROS TF listener.
    
    laser_assembler::AssembleScans assembleScanServiceMsg_;     ///< assemble scan service message.
    laser_geometry::LaserProjection projector_;                 ///< object to transform LaserScan data.

    PointCloudList cloudsInWindow;    // TOTO eliminate this variable?
    std::vector<sensor_msgs::PointCloud2*> current_clusters_;

    Eigen::Vector3d plane_normal_;             ///< XYZ plane normal.

    /**
    *  @brief callback for LegArray message
    *  @param legArrayMsg the LegArray message.
    */
    void legArrayCallback(const player_tracker::LegArray::ConstPtr &legArrayMsg);

    /**
    *  @brief callback for laser scan message
    *  @param scanMsg the laser scan message.
    */
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &scanMsg);

};
}


#endif // PLAYER_TRACKER_SPATIAL_TEMPORAL_H_