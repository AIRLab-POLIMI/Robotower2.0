#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// ROS messages
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
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
#include <player_tracker/Leg.h>

#include <sensor_msgs/point_cloud2_iterator.h>


// Custom messages

#include <cstddef>
#include <iostream>
#include <limits>
#include <vector>
#include <algorithm>    // std::min
#include <math.h>


#define OBSTACLE 100
#define UNKNOWN -1
#define ANGULAR_SPEED_TANH_TRESHOLD 1.5
#define ANGULAR_SPEED_IGNORE_TRESHOLD 0.8
#define ANGLE_ADJUSTMENT 0.0872665
#define LINEAR_SPEED_TANH_TRESHOLD 0.5
#define LINEAR_SPEED_IGNORE_TRESHOLD 0.1
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
	vel_sub_ = nh_.subscribe("/vel", 1, &OccupancyGridMapping::velocityCallback, this);
	detected_leg_sub_ = nh_.subscribe("/detected_leg_clusters", 1, &OccupancyGridMapping::detectedClusterCallback, this);
	cloud_map_pub_ = nh_.advertise<sensor_msgs::PointCloud>( "cloud_map", 1);
	player_pos_pub_ = nh_.advertise<visualization_msgs::Marker>( "player", 1);
	cloud_pub_obstacles_ = nh_.advertise<sensor_msgs::PointCloud>( "cloud_obstacles", 1);
	scan_player_pub_ = nh_.advertise<sensor_msgs::LaserScan>( "scan_player_tracking", 1);
	map_sub_ = nh.subscribe("map", 10, &OccupancyGridMapping::mapCallback, this);

	isteff_test_pub_ = nh_.advertise<sensor_msgs::PointCloud>( "isteff", 1);
	detected_leg_mask_.resize(1000, false);
  }

private:
	std::string scanTopic_;
	std::string fixed_frame_;
	std::string base_frame_;

	ros::NodeHandle nh_;

	ros::Subscriber laserScanSubscriber_;
	ros::Subscriber detected_leg_sub_;
	ros::Subscriber vel_sub_;
	ros::Subscriber map_sub_;

	ros::Publisher cloud_pub_obstacles_;
	ros::Publisher cloud_map_pub_;
	ros::Publisher scan_player_pub_;
	ros::Publisher isteff_test_pub_;
	ros::Publisher player_pos_pub_;

	nav_msgs::OccupancyGridPtr last_map_;

	geometry_msgs::Point32 current_pos_;

	geometry_msgs::Vector3 current_vel_wrt_map_;
	geometry_msgs::Vector3 current_linear_vel_;
	geometry_msgs::Vector3 current_angular_vel_;

	std::vector<player_tracker::Leg> leg_array_;
	std::vector<player_tracker::Leg> possible_player_legs_;
	double current_speed_;
	float current_direction_wrt_map_;
	float displacement_magnitude_;


	float current_rotation_wrt_map_;

	std::vector<bool> detected_leg_mask_;


	int width_;
	int min_points_per_cluster_;

	tf::TransformListener tfl_;

	void mapCallback(const nav_msgs::OccupancyGridPtr &msg){
		last_map_ = msg;
	}

	void detectedClusterCallback(const player_tracker::LegArray legs){
		leg_array_.resize(legs.legs.size());
		leg_array_ = legs.legs;

		detected_leg_mask_.clear();
		detected_leg_mask_.resize(1000, false);
		// ROS_INFO_STREAM(detected_leg_mask_)
		for(int i=0; i<legs.legs.size(); i++){
			for(int j=0; j<legs.legs[i].point_indexes.size(); j++){
				detected_leg_mask_[legs.legs[i].point_indexes[j]] = true;
			}
		}
	}

	void laserCallback(const sensor_msgs::LaserScan::ConstPtr &scanMsg){
		// do nothing in case we do not receive a map;
		if (last_map_ == nullptr or leg_array_.size() == 0){
			return;
		}

		// UPDATING ROBOT INFORMATION NECESSARY FOR POINT TRANSFORMATION
		updateCurrentPos();
		possible_player_legs_.clear();
		// updateCurrentMotionDirection();
		// calculateDisplacementMagnitude();
		geometry_msgs::Vector3 displacement_vector = calculateDisplacementVector();
		double rotation = calculateRotation();

		double angle_increment = 2*M_PI / scanMsg->ranges.size();
		sensor_msgs::LaserScan output_scan;
		sensor_msgs::PointCloud cloud_wrt_map;
		sensor_msgs::PointCloud cloud_obstacle;
		sensor_msgs::PointCloud cloud_isteff;
		std::vector<geometry_msgs::Point32> points_wrt_map;
		std::vector<geometry_msgs::Point32> obstacles;
		std::vector<geometry_msgs::Point32> isteff_points;
		points_wrt_map.resize(scanMsg->ranges.size());
		obstacles.resize(scanMsg->ranges.size());
		isteff_points.resize(scanMsg->ranges.size());

		output_scan.header = scanMsg->header;
		output_scan.ranges.resize(scanMsg->ranges.size());
		output_scan.angle_min = scanMsg->angle_min;
		output_scan.angle_max = scanMsg->angle_max;
		output_scan.angle_increment = scanMsg->angle_increment;
		output_scan.time_increment = scanMsg->time_increment;
		output_scan.scan_time = scanMsg->scan_time;
		output_scan.range_min = scanMsg->range_min;
		output_scan.range_max = scanMsg->range_max;

		unsigned int cellX_map = - last_map_->info.origin.position.x / last_map_->info.resolution;
		unsigned int cellY_map = - last_map_->info.origin.position.y / last_map_->info.resolution;

		cloud_wrt_map.header.frame_id = "/map";
		cloud_obstacle.header.frame_id = "/map";
		cloud_obstacle.header.stamp = ros::Time::now();

		cloud_isteff.header.frame_id = "/map";
		cloud_isteff.header.stamp = ros::Time::now();

		for(int i=0; i<leg_array_.size(); i++){
			// Take one leg cluster
			player_tracker::Leg leg = leg_array_[i];
			// Iterate trough the laser scan associated
			bool is_valid = true;
			for(int j=0; j<leg.point_indexes.size(); j++){
				// Transform the point to the map frame
				int scan_index = leg.point_indexes[j];
				double scan_angle = (scan_index * angle_increment) - M_PI;

				geometry_msgs::PointStamped point_wrt_robot;
				point_wrt_robot.point.x = cos(scan_angle + current_rotation_wrt_map_)*scanMsg->ranges[scan_index];
				point_wrt_robot.point.y = sin(scan_angle + current_rotation_wrt_map_)*scanMsg->ranges[scan_index];
				point_wrt_robot.point.z = 0.0;


				geometry_msgs::Point32 my_point; // PointCloud wants a Point32

				my_point = myTransformToMap(point_wrt_robot);
				// points_wrt_map.push_back(my_point);


				// Decode its position in the costmap
				unsigned int cellX_wrt_map = (my_point.x) / last_map_->info.resolution;
				unsigned int cellY_wrt_map = (my_point.y) / last_map_->info.resolution;

				unsigned int grid_x = (unsigned int)((my_point.x - last_map_->info.origin.position.x) / last_map_->info.resolution);
				unsigned int grid_y = (unsigned int)((my_point.y - last_map_->info.origin.position.y) / last_map_->info.resolution);

				int cellX = cellX_wrt_map + cellX_map;
				int cellY = cellY_wrt_map + cellY_map;
					
				if(checkContour(cellX, cellY)){
					// There's something on the map at this position
					//ROS_ERROR("OBSTACLE");
					obstacles[scan_index] = my_point;
					output_scan.ranges[scan_index] = std::numeric_limits<double>::infinity();
					// ROS_INFO("INVALID LEG");
					is_valid = false;
					break;
				}
				else{
					// NOT AN OBSTACLE ADD TO VISUALIZATION
					// points_wrt_map[i] = my_point;
					output_scan.ranges[scan_index] = scanMsg->ranges[scan_index];
					// if(detected_leg_mask_[i]){
					// 		// A leg was detected in that scan index
					// 	}
					// 	else{
					// 		output_scan.ranges[i] = std::numeric_limits<double>::infinity();
					// 	}
				}
			}
			if(is_valid){
				ROS_WARN("VALID LEG");
				possible_player_legs_.push_back(leg);
			}
		}

		// if(possible_player_legs_.size() >= 2){
		// 	// geometry_msgs::Point32 position;

		// 	// position.x = (possible_player_legs_[0].position.x + possible_player_legs_[1].position.x) / 2;
		// 	// position.y = (possible_player_legs_[0].position.y + possible_player_legs_[1].position.y) / 2;
		// 	// position.z = 0.0;

		// 	publishPlayerPos(possible_player_legs_[0], possible_player_legs_[1]);
		// }
		if(possible_player_legs_.size() >= 2){
			// Find pair with suitable distance
			for(int h=0; h<possible_player_legs_.size() - 1; h++){
				for(int k = h+1; k<possible_player_legs_.size(); k++){
					float pair_distance = sqrt(pow(possible_player_legs_[h].position.x - possible_player_legs_[k].position.x, 2) + pow(possible_player_legs_[h].position.y - possible_player_legs_[k].position.y, 2));
					ROS_WARN("DISTANCE %.2f ", pair_distance);
					if(pair_distance < 1){
						ROS_INFO("SUITABLE PAIR");
						publishPlayerPos(possible_player_legs_[h], possible_player_legs_[k]);
					}
				}
			}
		}

		// for (unsigned int i=0; i < scanMsg->ranges.size(); i++){

		// 	if (scanMsg->ranges[i] < 6.6){
		// 		// Angle of scan wrt robot
		// 		// Get in range (-pi, +pi)
		// 		double scan_angle = (i * angle_increment) - M_PI;
		// 		geometry_msgs::PointStamped point_wrt_robot;
		// 		point_wrt_robot.header = scanMsg->header;
		// 		point_wrt_robot.point.x = cos(scan_angle + current_rotation_wrt_map_)*scanMsg->ranges[i];
		// 		point_wrt_robot.point.y = sin(scan_angle + current_rotation_wrt_map_)*scanMsg->ranges[i];
		// 		point_wrt_robot.point.z = 0.0;

		// 		geometry_msgs::PointStamped point_wrt_map;
		// 		geometry_msgs::Point32 my_point_wrt_map;

		// 		geometry_msgs::Point32 my_point; // PointCloud wants a Point32

		// 		my_point = myTransformToMap(point_wrt_robot);
		// 		points_wrt_map.push_back(my_point);
				
		// 		my_point = traslate(my_point, displacement_vector);
		// 		// my_point = rotatePoint(my_point, rotation);
		// 		isteff_points.push_back(my_point);

		// 		unsigned int cellX_wrt_map = (my_point.x) / last_map_->info.resolution;
		// 		unsigned int cellY_wrt_map = (my_point.y) / last_map_->info.resolution;

		// 		unsigned int grid_x = (unsigned int)((my_point.x - last_map_->info.origin.position.x) / last_map_->info.resolution);
		// 		unsigned int grid_y = (unsigned int)((my_point.y - last_map_->info.origin.position.y) / last_map_->info.resolution);

		// 		int cellX = cellX_wrt_map + cellX_map;
		// 		int cellY = cellY_wrt_map + cellY_map;
					
		// 		if(checkContour(cellX, cellY)){
		// 			// There's something on the map at this position
		// 			//ROS_ERROR("OBSTACLE");
		// 			obstacles[i] = my_point;
		// 			output_scan.ranges[i] = std::numeric_limits<double>::infinity();
		// 		}
		// 		else{
		// 			// NOT AN OBSTACLE ADD TO VISUALIZATION
		// 			// points_wrt_map[i] = my_point;
		// 			if(detected_leg_mask_[i]){
		// 					// A leg was detected in that scan index
		// 					output_scan.ranges[i] = scanMsg->ranges[i];
		// 				}
		// 				else{
		// 					output_scan.ranges[i] = std::numeric_limits<double>::infinity();
		// 				}
		// 		}
		// 	}
		// 	else{
		// 		output_scan.ranges[i] = std::numeric_limits<double>::infinity();
		// 	}
		// }
		cloud_wrt_map.points = points_wrt_map;
		cloud_obstacle.points = obstacles;
		cloud_isteff.points = isteff_points;

		// cloud_map_pub_.publish(cloud_wrt_map); // Publishing reconstructed map from laser
		cloud_pub_obstacles_.publish(cloud_obstacle);
		isteff_test_pub_.publish(cloud_isteff);
		scan_player_pub_.publish(output_scan);
	}

	void publishPlayerPos(player_tracker::Leg leg1, player_tracker::Leg leg2){
		ROS_INFO("PUBLISHING PLAYER POS");
		geometry_msgs::Point32 pos;
		pos.x = (leg1.position.x + leg2.position.x) / 2;
		pos.y = (leg1.position.y + leg2.position.y) / 2;
		pos.z = 0.0;

		visualization_msgs::Marker marker;
		marker.header.frame_id = "/map";
		marker.header.stamp = ros::Time::now();

		marker.type = visualization_msgs::Marker::SPHERE;
		
		marker.scale.x = 0.2;
		marker.scale.y = 0.2;
		marker.scale.z = 0.2;
		
		marker.color.r = 0.0f;
		marker.color.g = 1.0f;
		marker.color.b = 0.0f;
		marker.color.a = 1.0f;

		marker.pose.position.x = pos.x;
		marker.pose.position.y = pos.y;
		marker.pose.position.z = pos.z;

		player_pos_pub_.publish(marker);
	}

	geometry_msgs::Vector3 calculateDisplacementVector(){
		/*
		Calculate the displacement 
		*/
		geometry_msgs::Vector3 displacement_vector;

		geometry_msgs::Vector3 velocity_wrt_map = rotateVector(current_linear_vel_, current_rotation_wrt_map_);

		// Rotate by 180° to account for opposite direction
		geometry_msgs::Vector3 temporary_displacement_vector = rotateVector(velocity_wrt_map, M_PI);
		float direction = atan2(temporary_displacement_vector.y, temporary_displacement_vector.x);

		// FIXED MAGNITUDE FOR TESTING
		if(current_speed_ > 0.0){
			double prop = tanhProportion(current_speed_, LINEAR_SPEED_TANH_TRESHOLD);
			displacement_vector.x = (prop*0.1) * cos(direction);
			displacement_vector.y = (prop*0.1) * sin(direction);
		}
		// displacement_vector.x = current_speed_ * cos(direction);
		// displacement_vector.y = current_speed_ * sin(direction);
		return displacement_vector;
	}

	double calculateRotation(){
		double intensity = 0.174533;
		double angular_speed = fabs(current_angular_vel_.z);
		double prop = tanhProportion(angular_speed, ANGULAR_SPEED_TANH_TRESHOLD);

		if(angular_speed > ANGULAR_SPEED_IGNORE_TRESHOLD){
			if(current_angular_vel_.z > 0){
				return -prop * ANGLE_ADJUSTMENT;
			}
			return prop * ANGLE_ADJUSTMENT;//-current_angular_vel_.z/3.0;
		}

		return 0.0;

	}

	double tanhProportion(double in, double max_value){
		double proportion = (in / max_value) - 1;
		if(proportion >= 0){
			return 1.0;
		}
		
		return (tanh(proportion) + 1);

	}

	geometry_msgs::Point32 traslate(geometry_msgs::Point32 in, geometry_msgs::Vector3 vector){
		geometry_msgs::Point32 out;

		out.x = in.x + vector.x;
		out.y = in.y + vector.y;
		out.z = 0.0;

		return out;
	}

	geometry_msgs::Point32 myTransformToMap(geometry_msgs::PointStamped point_wrt_robot){
		geometry_msgs::Point32 my_point; // PointCloud wants a Point32
		my_point.x = point_wrt_robot.point.x + current_pos_.x;
		my_point.y = point_wrt_robot.point.y + current_pos_.y;

		return my_point;
	}

	void velocityCallback(const geometry_msgs::Twist vel){
		/*
			Callback for robot velocity
		*/
		double speed = sqrt(pow(vel.linear.x, 2) + pow(vel.linear.y, 2));
		if(speed < LINEAR_SPEED_IGNORE_TRESHOLD){
			current_speed_ = 0.0;
			current_linear_vel_.x = 0.0;
			current_linear_vel_.y = 0.0;
			current_linear_vel_.z = 0.0;
		}else{
			current_speed_ = speed;
			current_linear_vel_ = vel.linear;
		}
		current_angular_vel_ = vel.angular;

	}

	geometry_msgs::Vector3 rotateVector(geometry_msgs::Vector3 in, float angle){
		geometry_msgs::Vector3 out;

		float current_orientation = atan2(in.y, in.x);
		float magnitude = sqrt(pow(in.x, 2) + pow(in.y, 2));
		float new_orientation = current_orientation + angle;

		out.x = magnitude * cos(new_orientation);
		out.y = magnitude * sin(new_orientation);
		out.z = 0.0;
		return out;
	}

	geometry_msgs::Point32 rotatePoint(geometry_msgs::Point32 in, float angle){
		geometry_msgs::Point32 out;

		float current_orientation = atan2(in.y, in.x);
		float distance = sqrt(pow(in.x, 2) + pow(in.y, 2));
		float new_orientation = current_orientation + angle;

		out.x = distance * cos(new_orientation);
		out.y = distance * sin(new_orientation);
		out.z = 0.0;
		return out;
	}

	// void updateCurrentMotionDirection(){
	// 	current_vel_wrt_map_ = rotateVector(current_vel_, current_rotation_wrt_map_);
	// 	current_direction_wrt_map_ = atan2(current_vel_wrt_map_.y, current_vel_wrt_map_.x);
	// }

	// void calculateDisplacementMagnitude(){
	// 	float velocity_magnitude = sqrt(pow(current_vel_.x, 2) + pow(current_vel_.y, 2));
	// 	if(velocity_magnitude < 0.1){
	// 		displacement_magnitude_ = 0;
	// 		return;
	// 	}
	// 	// TODO ADJUST THIS FUNCTION TO A PROPER ONE
	// 	displacement_magnitude_ = velocity_magnitude;
	// }

	// geometry_msgs::Point32 transalatePointAccountingVelocity(geometry_msgs::Point32 in){
	// 	geometry_msgs::Point32 out;

	// 	// We need to move in the opposite direction of motion
	// 	float displacement_magnitude_x = displacement_magnitude_ * cos(current_direction_wrt_map_ - M_PI);
	// 	float displacement_magnitude_y = displacement_magnitude_ * sin(-current_direction_wrt_map_ - M_PI);

	// 	out.x = in.x + displacement_magnitude_x;
	// 	out.y = in.y + displacement_magnitude_y;
	// 	out.z = 0;
	// 	return out;
	// }

	bool checkContour(unsigned int cellX, unsigned int cellY){
		float tolerance_radius = 0.35; // Tolerating misplacement of 35cm
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
