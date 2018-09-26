#include <player_tracker/laser_cluster_extractor.h>

LaserClusterExtractor::LaserClusterExtractor(){

    nh_.param("scan_topic", scan_topic_, std::string("scan"));
    nh_.param("fixed_frame", fixed_frame_, std::string("base_link"));
    nh_.param("use_scan_header_stamp_for_tfs", use_scan_header_stamp_for_tfs_, false);
    nh_.param("cluster_dist_euclid", cluster_dist_euclid_, 0.13);
    nh_.param("max_detect_distance", max_detect_distance_, 10.0);
    nh_.param("min_points_per_cluster", min_points_per_cluster_, 3);  
    laser_scan_subscriber_ =  nh_.subscribe(scan_topic_, 10, &LaserClusterExtractor::laserCallback, this);
    detected_laser_clusters_pub_ = nh_.advertise<player_tracker::ClusterArray>("detected_laser_clusters",20);
}


/**
  * @brief Comparison class to order clusters according to their relative distance to the laser scanner
  */
  class CompareClusters
  {
  public:
      bool operator ()(const player_tracker::Cluster &a, const player_tracker::Cluster &b)
      {
          float rel_dist_a = pow(a.position.x*a.position.x + a.position.y*a.position.y, 1./2.);
          float rel_dist_b = pow(b.position.x*b.position.x + b.position.y*b.position.y, 1./2.);          
          return rel_dist_a < rel_dist_b;
      }
  };

void LaserClusterExtractor::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    laser_processor::ScanProcessor processor(*scan); 
    processor.splitConnected(cluster_dist_euclid_);
    processor.removeLessThan(min_points_per_cluster_);


    player_tracker::ClusterArray clusters;
    clusters.header.frame_id = scan->header.frame_id;
    clusters.header.stamp = scan->header.stamp;

    // Find out the time that should be used for tfs
    bool transform_available;
    ros::Time tf_time;

    // Use time from scan header
    if (use_scan_header_stamp_for_tfs_){
        tf_time = scan->header.stamp;

        try{
            tfl_.waitForTransform(fixed_frame_, scan->header.frame_id, tf_time,
                                    ros::Duration(1.0));
            transform_available =
                tfl_.canTransform(fixed_frame_, scan->header.frame_id, tf_time);
        }catch(tf::TransformException ex){
            ROS_INFO("Detect_leg_clusters: No tf available");
            transform_available = false;
        }
    
    }else{
        // Otherwise just use the latest tf available
        tf_time = ros::Time(0);
        transform_available =
            tfl_.canTransform(fixed_frame_, scan->header.frame_id, tf_time);
    }

    // Store all processes legs in a set ordered according to their relative
    // distance to the laser scanner
    std::set<player_tracker::Cluster, CompareClusters> cluster_set;
    if (!transform_available) {
        ROS_INFO("Not publishing detected leg clusters because no tf was available");
    } else{  // transform_available
        // Iterate through all clusters
        for (std::list<laser_processor::SampleSet*>::iterator cluster = processor.getClusters().begin();
            cluster != processor.getClusters().end(); cluster++){   
            
            // Get position of cluster in laser frame
            tf::Stamped<tf::Point> position((*cluster)->getPosition(), tf_time, scan->header.frame_id);
            float rel_dist = pow(position[0]*position[0] + position[1]*position[1], 1./2.);

            // Only consider clusters within max_distance. 
            if (rel_dist < max_detect_distance_){

                // Transform cluster position to fixed frame
                // This should always be succesful because we've checked earlier if a tf was available
                bool transform_successful_2;
                try{
                    tfl_.transformPoint(fixed_frame_, position, position);
                    transform_successful_2 = true;
                }catch (tf::TransformException ex){
                    ROS_ERROR("%s",ex.what());
                    transform_successful_2 = false;
                }

                if (transform_successful_2){  
                    // Add detected cluster to set of detected clusters, along with its relative position to the laser scanner
                    player_tracker::Cluster new_cluster;
                    new_cluster.position.x = position[0];
                    new_cluster.position.y = position[1];
                    new_cluster.points = (*cluster)->getSamples();
                    new_cluster.point_indexes = (*cluster)->getSampleIndexes();
                    cluster_set.insert(new_cluster);
                }
            }
        }
    }

    // Publish detected clusters to /detected_laser_clusters
    // They are ordered from closest to the laser scanner to furthest    
    for (std::set<player_tracker::Cluster>::iterator it = cluster_set.begin(); it != cluster_set.end(); ++it){
      // Publish to /detected_leg_clusters topic
      player_tracker::Cluster cluster = *it;
      clusters.clusters.push_back(cluster);
    }

    detected_laser_clusters_pub_.publish(clusters);

}


int main(int argc, char **argv){
  ros::init(argc, argv, "laser_cluster_extractor");
  LaserClusterExtractor lce;
  ros::spin();
  return 0;
}
