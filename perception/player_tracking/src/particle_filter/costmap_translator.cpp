#include <ros/ros.h>
#include <tf/transform_listener.h>

// ROS messages
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/shared_ptr.hpp>

#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/observation_buffer.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <visualization_msgs/Marker.h>
#include <player_tracker/LegArray.h>

// Custom messages
#include <cstddef>
#include <iostream>
#include <limits>
#include <vector>


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
    nh_.param("fixed_frame", fixedFrame_, std::string("/map"));
    nh_.param("base_frame", baseFrame_, std::string("/base_link"));
    // Initialize map
	lastMap_ = nullptr;
	

	laserScanSubscriber_ = nh_.subscribe(scan_topic, 1, &OccupancyGridMapping::laserCallback, this);
	legArraySubscriber_ = nh_.subscribe("/detected_leg_clusters", 1, &OccupancyGridMapping::legArrayCallback, this);
	playerPointCloudPublisher = nh_.advertise<sensor_msgs::PointCloud>( "player_cloud", 1);
	scanPlayerPublisher_ = nh_.advertise<sensor_msgs::LaserScan>( "scan_player_tracking", 1);
	mapSubscriber_ = nh.subscribe("map", 10, &OccupancyGridMapping::mapCallback, this);

	legMask_.resize(1000, false);


  }

private:
	std::string scanTopic_;
	std::string fixedFrame_;
	std::string baseFrame_;

	ros::NodeHandle nh_;

	ros::Subscriber laserScanSubscriber_;
	ros::Subscriber legArraySubscriber_;
	ros::Subscriber mapSubscriber_;

	ros::Publisher scanPlayerPublisher_;
	ros::Publisher playerPointCloudPublisher;

	nav_msgs::OccupancyGridPtr lastMap_;

	geometry_msgs::Point32 currentPos_;
	float currentRotationWrtMapFrame_;

	std::vector<bool> legMask_;

	tf::TransformListener tfl_;

	void mapCallback(const nav_msgs::OccupancyGridPtr &msg){
		lastMap_ = msg;
	}

	void legArrayCallback(const player_tracker::LegArray legs){
		// Builds a mask to filter scan indexes where we found something of a shape of cylinder
		legMask_.clear();
		legMask_.resize(1000, false);
		for(int i=0; i<legs.legs.size(); i++){
			for(int j=0; j<legs.legs[i].point_indexes.size(); j++){
				legMask_[legs.legs[i].point_indexes[j]] = true;
			}
		}
	}

	sensor_msgs::LaserScan copyLaserScanInfo(sensor_msgs::LaserScan scanMsg){
		sensor_msgs::LaserScan newScanMsg;

		newScanMsg.header = scanMsg.header;
		newScanMsg.ranges.resize(scanMsg.ranges.size());
		newScanMsg.angle_min = scanMsg.angle_min;
		newScanMsg.angle_max = scanMsg.angle_max;
		newScanMsg.angle_increment = scanMsg.angle_increment;
		newScanMsg.time_increment = scanMsg.time_increment;
		newScanMsg.scan_time = scanMsg.scan_time;
		newScanMsg.range_min = scanMsg.range_min;
		newScanMsg.range_max = scanMsg.range_max;

		return newScanMsg;
	}

	geometry_msgs::Point32 transformPointToMapFrame(float angle, float distance){
		geometry_msgs::Point32 pointInMapFrame;

		pointInMapFrame.x = cos(angle + currentRotationWrtMapFrame_)*distance + currentPos_.x;
		pointInMapFrame.y = sin(angle + currentRotationWrtMapFrame_)*distance + currentPos_.y;

		return pointInMapFrame;
	}

	void laserCallback(const sensor_msgs::LaserScan scanMsg){
		// do nothing in case we do not receive a map;
		if (lastMap_ == nullptr){
			return;
		}

		updateCurrentPos();
		double angleIncrement = 2*M_PI / scanMsg.ranges.size();
		sensor_msgs::LaserScan playerLaserScan = copyLaserScanInfo(scanMsg);

		sensor_msgs::PointCloud playerPointCloud;
		std::vector<geometry_msgs::Point32> playerPoints;


		unsigned int originMapX = - lastMap_->info.origin.position.x / lastMap_->info.resolution;
		unsigned int originMapY = - lastMap_->info.origin.position.y / lastMap_->info.resolution;

		playerPointCloud.header.frame_id = "/map"; // Point cloud with points associated to the player position
		playerPointCloud.header.stamp = ros::Time::now();

		for (unsigned int i=0; i < scanMsg.ranges.size(); i++){
			if(legMask_[i]){
				// A leg was detected in that scan index, it could be a player
				if (scanMsg.ranges[i] < 6.6){
					double scan_angle = (i * angleIncrement) - M_PI;
					geometry_msgs::Point32 pointInMapFrame = transformPointToMapFrame(scan_angle, scanMsg.ranges[i]);
					
					unsigned int cellXCoordinateWithRespectToMapOrigin = (pointInMapFrame.x) / lastMap_->info.resolution;
					unsigned int cellYCoordinateWithRespectToMapOrigin = (pointInMapFrame.y) / lastMap_->info.resolution;

					int cellXCoordinateAbsolut = cellXCoordinateWithRespectToMapOrigin + originMapX;
					int cellYCoordinateAbsolut = cellYCoordinateWithRespectToMapOrigin + originMapY;
				
					bool isFixedObstaclePresent = checkContour(cellXCoordinateAbsolut, cellYCoordinateAbsolut);
					if(!isFixedObstaclePresent){
						playerPoints.push_back(pointInMapFrame);
						playerLaserScan.ranges[i] = scanMsg.ranges[i];
					}
					else{
						playerLaserScan.ranges[i] = std::numeric_limits<double>::infinity();
					}
				}
			}
			else{
				playerLaserScan.ranges[i] = std::numeric_limits<double>::infinity();
			}
		}
		playerPointCloud.points = playerPoints;
		playerPointCloudPublisher.publish(playerPointCloud);
		scanPlayerPublisher_.publish(playerLaserScan);
	}

	bool checkContour(unsigned int cellXCoordinateAbsolut, unsigned int cellYCoordinateAbsaolut){
		float tolerance_radius = TOLERANCE_RADIUS; // Tolerating misplacement of 35cm
		int tolerance_cell_number = tolerance_radius / lastMap_->info.resolution;
		int number_of_cells_to_check = pow(tolerance_cell_number, 2);

		int starting_cell_x = cellXCoordinateAbsolut - tolerance_cell_number/2;
		int starting_cell_y = cellYCoordinateAbsaolut - tolerance_cell_number/2;

		for(int i=0; i<tolerance_cell_number; i++){
			for(int j=0; j<tolerance_cell_number; j++){
				int cellToCheckX = starting_cell_x + j;
				int cellToCheckY = starting_cell_y + i;

				// clamping the values to avoid segmentation fault

				int max_x = (int) lastMap_->info.width -1;
				int max_y = (int) lastMap_->info.height -1;

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

	bool checkObstacle(unsigned int cellXCoordinateAbsolut, unsigned int cellYCoordinateAbsolut){

		int idx = cellYCoordinateAbsolut * lastMap_->info.width + cellXCoordinateAbsolut;

		// TODO: this condition is used to avoid the index becoming negative.
		// This is obviously a workaround. I suspect the problem comes from a kind
		// of overflow in one of the variables used to compute it, thus, forcing the
		// appearance of a negative value. This is grounded by the standard behavior
		// in overflow conditions. 
		if (idx < 0){
			return false;
		}

		if (lastMap_->data[idx] == OBSTACLE || lastMap_->data[idx] == UNKNOWN){
				return true;
			}
		return false;
	}

	void updateCurrentPos(){
        /**
		Updates the robot position with respect to the map frame
		Updates x,y coordinates e and current rotation
		**/
        tf::StampedTransform transform;
        try{
            tfl_.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(10.0));
            tfl_.lookupTransform("/map", "/base_link", ros::Time(0), transform);
            currentPos_.x = transform.getOrigin().getX();
            currentPos_.y = transform.getOrigin().getY();
            currentPos_.z = transform.getOrigin().getZ();


            double roll, pitch, yaw;
            tf::Quaternion quat = transform.getRotation();            
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            
            currentRotationWrtMapFrame_ = yaw;
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
