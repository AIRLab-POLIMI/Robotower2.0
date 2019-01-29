#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// ROS messages
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/shared_ptr.hpp>
#include <laser_geometry/laser_geometry.h>

#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/observation_buffer.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <visualization_msgs/Marker.h>
#include <laser_geometry/laser_geometry.h>

#include <player_tracker/LegArray.h>

#include <sensor_msgs/point_cloud2_iterator.h>

// Custom messages

#include <cstddef>
#include <iostream>
#include <limits>
#include <vector>
#include <algorithm>    // std::min


#define TOLERANCE_RADIUS 0.3
#define OBSTACLE 100
#define UNKNOWN -1

/**
* @basic A simple 'local' occupancy grid map that maps LaserScan points that are not part
* of the original map.
* 
* The occupied areas on the map are all moving objects.
*/
class OccupancyGridMapping{
public:
	/**
	* @basic Constructor
	* @param nh A nodehandle
	* @param scan_topic The topic for the scan we would like to map
	*/
	OccupancyGridMapping(ros::NodeHandle nh, std::string scan_topic): nh_(nh), 
						 scanTopic_(scan_topic){

    ros::NodeHandle nh_private("~");
    std::string local_map_topic;
    nh_.param("fixed_frame", fixed_frame_, std::string("/map"));
    nh_.param("base_frame", base_frame_, std::string("/base_link"));
    // Initialize map
	last_map_ = nullptr;
	

	laserScanSubscriber_ = nh_.subscribe(scan_topic, 1, &OccupancyGridMapping::laserCallback, this);
	detected_leg_sub_ = nh_.subscribe("/detected_leg_clusters", 1, &OccupancyGridMapping::detectedClusterCallback, this);
	// cloud_pub_isteff_ = nh_.advertise<sensor_msgs::PointCloud>( "test_cloud_isteff", 1);
	// cloud_pub_obstacles_ = nh_.advertise<sensor_msgs::PointCloud>( "cloud_obstacles", 1);
	// scan_player_pub_tmp_ = nh_.advertise<sensor_msgs::LaserScan>( "scan_player_tracking_tmp", 1);
	scan_player_pub_ = nh_.advertise<sensor_msgs::LaserScan>( "scan_player_tracking", 1);
	map_sub_ = nh.subscribe("map", 10, &OccupancyGridMapping::mapCallback, this);

	detected_leg_mask_.resize(1000, false);


  }

private:
	std::string scanTopic_;
	std::string fixed_frame_;
	std::string base_frame_;

	ros::NodeHandle nh_;

	ros::Subscriber laserScanSubscriber_;
	ros::Subscriber legClusterSubscriber_;
	ros::Subscriber detected_leg_sub_;
	ros::Subscriber map_sub_;

	// ros::Publisher cloud_pub_obstacles_;
	ros::Publisher scan_player_pub_;
	// ros::Publisher scan_player_pub_tmp_;

	// ros::Publisher cloud_pub_isteff_;

	nav_msgs::OccupancyGridPtr last_map_;

	geometry_msgs::Point32 current_pos_;


	float current_rotation_wrt_map_;

	std::vector<bool> detected_leg_mask_;


	int width_;
	int min_points_per_cluster_;

	tf::TransformListener tfl_;

	void mapCallback(const nav_msgs::OccupancyGridPtr &msg){
		last_map_ = msg;
	}

	void detectedClusterCallback(const player_tracker::LegArray legs){
		detected_leg_mask_.clear();
		detected_leg_mask_.resize(1000, false);
		// ROS_INFO_STREAM(detected_leg_mask_)
		for(int i=0; i<legs.legs.size(); i++){
			for(int j=0; j<legs.legs[i].point_indexes.size(); j++){
				detected_leg_mask_[legs.legs[i].point_indexes[j]] = true;
			}
		}
	}

	// sensor_msgs::LaserScan filterObstacles(sensor_msgs::LaserScan scan_obstacles){
	// 	// Removes evidences outside the static map that are not legs
	// 	int i;
	// 	for(i=0; i<scan_obstacles.ranges.size(); i++){
	// 		if(scan_obstacles.ranges[i] != std::numeric_limits<double>::infinity()){
	// 			if(!detected_leg_mask_[i]){
	// 				// If the index does not correspond to a leg cluster
	// 				scan_obstacles.ranges[i] = std::numeric_limits<double>::infinity();
	// 			}
	// 		}
	// 	}

	// 	return scan_obstacles;
	// }

	void laserCallback(const sensor_msgs::LaserScan::ConstPtr &scanMsg){
		// do nothing in case we do not receive a map;
		if (last_map_ == nullptr){
			return;
		}

		updateCurrentPos();
		double angle_increment = 2*M_PI / scanMsg->ranges.size();
		sensor_msgs::LaserScan output_scan;
		// sensor_msgs::LaserScan output_scan_tmp;
		sensor_msgs::PointCloud cloud_wrt_map;
		sensor_msgs::PointCloud cloud_obstacle;
		std::vector<geometry_msgs::Point32> points_wrt_map;
		std::vector<geometry_msgs::Point32> obstacles;
		points_wrt_map.resize(scanMsg->ranges.size());
		obstacles.resize(scanMsg->ranges.size());

		output_scan.header = scanMsg->header;
		output_scan.ranges.resize(scanMsg->ranges.size());
		output_scan.angle_min = scanMsg->angle_min;
		output_scan.angle_max = scanMsg->angle_max;
		output_scan.angle_increment = scanMsg->angle_increment;
		output_scan.time_increment = scanMsg->time_increment;
		output_scan.scan_time = scanMsg->scan_time;
		output_scan.range_min = scanMsg->range_min;
		output_scan.range_max = scanMsg->range_max;

		// Scan to get an image for my thesis [ISTEFF]
		// output_scan_tmp.header = scanMsg->header;
		// output_scan_tmp.ranges.resize(scanMsg->ranges.size());
		// output_scan_tmp.angle_min = scanMsg->angle_min;
		// output_scan_tmp.angle_max = scanMsg->angle_max;
		// output_scan_tmp.angle_increment = scanMsg->angle_increment;
		// output_scan_tmp.time_increment = scanMsg->time_increment;
		// output_scan_tmp.scan_time = scanMsg->scan_time;
		// output_scan_tmp.range_min = scanMsg->range_min;
		// output_scan_tmp.range_max = scanMsg->range_max;

		// ROS_INFO("ORGIN X:%.2f, Y:%.2f", last_map_->info.origin.position.x, last_map_->info.origin.position.y);
		//ROS_INFO("RESOLUTION X:%.2f", last_map_->info.resolution);
		unsigned int cellX_map = - last_map_->info.origin.position.x / last_map_->info.resolution;
		unsigned int cellY_map = - last_map_->info.origin.position.y / last_map_->info.resolution;

		cloud_wrt_map.header.frame_id = "/map";
		cloud_obstacle.header.frame_id = "/map";
		cloud_obstacle.header.stamp = ros::Time::now();

		for (unsigned int i=0; i < scanMsg->ranges.size(); i++){

			if (scanMsg->ranges[i] < 6.6){
				// Angle of scan wrt robot
				// Get in range (-pi, +pi)
				double scan_angle = (i * angle_increment) - M_PI;
				geometry_msgs::PointStamped point_wrt_robot;
				point_wrt_robot.header = scanMsg->header;
				point_wrt_robot.point.x = cos(scan_angle + current_rotation_wrt_map_)*scanMsg->ranges[i];
				point_wrt_robot.point.y = sin(scan_angle + current_rotation_wrt_map_)*scanMsg->ranges[i];
				point_wrt_robot.point.z = 0.0;

				geometry_msgs::PointStamped point_wrt_map;
				geometry_msgs::Point32 my_point_wrt_map;

				try {
					// tfl_.waitForTransform(fixed_frame_, scanMsg->header.frame_id, ros::Time(0), ros::Duration(10.0));
					// tfl_.transformPoint(fixed_frame_, point_wrt_robot, point_wrt_map);
					point_wrt_map.point.x = point_wrt_robot.point.x + current_pos_.x;
					point_wrt_map.point.y = point_wrt_robot.point.y + current_pos_.y;

					geometry_msgs::Point32 my_point; // PointCloud wants a Point32
					my_point.x = point_wrt_map.point.x;
					my_point.y = point_wrt_map.point.y;

					unsigned int cellX_wrt_map = (my_point.x) / last_map_->info.resolution;
					unsigned int cellY_wrt_map = (my_point.y) / last_map_->info.resolution;

					unsigned int grid_x = (unsigned int)((my_point.x - last_map_->info.origin.position.x) / last_map_->info.resolution);
					unsigned int grid_y = (unsigned int)((my_point.y - last_map_->info.origin.position.y) / last_map_->info.resolution);

					int cellX = cellX_wrt_map + cellX_map;
					int cellY = cellY_wrt_map + cellY_map;
					
					if(checkContour(cellX, cellY)){
						// There's something on the map at this position
						//ROS_ERROR("OBSTACLE");
						obstacles[i] = my_point;
						output_scan.ranges[i] = std::numeric_limits<double>::infinity();
						// output_scan_tmp.ranges[i] = std::numeric_limits<double>::infinity();
					}
					else{
						// NOT AN OBSTACLE ADD TO VISUALIZATION
						points_wrt_map[i] = my_point;
						// output_scan_tmp.ranges[i] = scanMsg->ranges[i];
						if(detected_leg_mask_[i]){
							// A leg was detected in that scan index
							output_scan.ranges[i] = scanMsg->ranges[i];
						}
						else{
							output_scan.ranges[i] = std::numeric_limits<double>::infinity();
						}
					}

				} catch (tf::TransformException ex) {
					// ROS_ERROR("Local map: No tf available. Details: %s", ex.what());
				}
			}
			else{
				output_scan.ranges[i] = std::numeric_limits<double>::infinity();
				// output_scan_tmp.ranges[i] = std::numeric_limits<double>::infinity();
			}
		}
		// cloud_wrt_map.points = points_wrt_map;
		// cloud_obstacle.points = obstacles;
		// cloud_pub_isteff_.publish(cloud_wrt_map); // Publishes player as PointCloud
		// cloud_pub_obstacles_.publish(cloud_obstacle);
		scan_player_pub_.publish(output_scan);
		// scan_player_pub_tmp_.publish(output_scan_tmp);
	}

	bool checkContour(unsigned int cellX, unsigned int cellY){
		float tolerance_radius = TOLERANCE_RADIUS; // Tolerating misplacement of 35cm
		int tolerance_cell_number = tolerance_radius / last_map_->info.resolution;
		int number_of_cells_to_check = pow(tolerance_cell_number, 2);

		int starting_cell_x = cellX - tolerance_cell_number/2;
		int starting_cell_y = cellY - tolerance_cell_number/2;

		for(int i=0; i<tolerance_cell_number; i++){
			for(int j=0; j<tolerance_cell_number; j++){
				int cellToCheckX = starting_cell_x + j;
				int cellToCheckY = starting_cell_y + i;

				// clamping the values to avoid segmentation fault

				int max_x = (int) last_map_->info.width -1;
				int max_y = (int) last_map_->info.height -1;

				cellToCheckX = std::min(cellToCheckX, max_x);
				cellToCheckY = std::min(cellToCheckY, max_y);

				bool isObstacle = checkObstacle(cellToCheckX, cellToCheckY);
				if (isObstacle){
					return true;
				}
			}
		}
		return false;
	}

	bool checkObstacle(unsigned int cellX, unsigned int cellY){

		int idx = cellY * last_map_->info.width + cellX;

		// TODO: this condition is used to avoid the index becoming negative.
		// This is obviously a workaround. I suspect the problem comes from a kind
		// of overflow in one of the variables used to compute it, thus, forcing the
		// appearance of a negative value. This is grounded by the standard behavior
		// in overflow conditions. 
		if (idx < 0){
			return false;
		}

		if (last_map_->data[idx] == OBSTACLE || last_map_->data[idx] == UNKNOWN){
				return true;
			}
		return false;
	}

	void updateCurrentPos(){
        /**
        Gets robot global position. That is, performs a TF transformation from /base_link to /map and returns
        x,y and theta.
        OUTPUTS:
        @ a 3D-numpy array defined as: [x, y, theta] w.r.t /map.
        **/
        tf::StampedTransform transform;
        try{
            tfl_.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(10.0));
            tfl_.lookupTransform("/map", "/base_link", ros::Time(0), transform);
            current_pos_.x = transform.getOrigin().getX();
            current_pos_.y = transform.getOrigin().getY();
            current_pos_.z = transform.getOrigin().getZ();


            double roll, pitch, yaw;
            tf::Quaternion quat = transform.getRotation();            
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            
            current_rotation_wrt_map_ = yaw;
            //ROS_INFO("Current rotation: %.2f", current_rotation_wrt_map_);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "costmap_translator");

	/** @todo We need to get a param, scan_topic, which is needed for the initialization
	list of OccupancyGridMapping. Is there a clearer way of doing this? */
	ros::NodeHandle nh;
	std::string scan_topic;
	nh.param("scan_topic", scan_topic, std::string("scan"));
	OccupancyGridMapping ogm(nh, scan_topic);

	ros::spin();
	return 0;
}
