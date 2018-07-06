#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// ROS messages
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

// Custom messages
#include <player_tracker/laser_processor.h>

/** @todo Make these parameters externally settable */
#define ALPHA 0.2
#define BETA 0.2 //0.1
#define OBSTACLE 1 //0.7
#define FREE_SPACE 0.2
#define UNKNOWN 0.5
#define MIN_PROB 0.001
#define MAX_PROB 1 - MIN_PROB

/**
* @basic A simple 'local' occupancy grid map that maps a small area around the robot.
* The occupied areas on the map are all obstacles.
*/
class OccupancyGridMapping {
public:
  /**
  * @basic Constructor
  * @param nh A nodehandle
  * @param scan_topic The topic for the scan we would like to map
  */
  OccupancyGridMapping(ros::NodeHandle nh, std::string scan_topic): nh_(nh), 
  		grid_centre_pos_found_(false), scanTopic_(scan_topic){

    ros::NodeHandle nh_private("~");
    std::string local_map_topic;
    nh_.param("fixed_frame", fixed_frame_, std::string("odom"));
    nh_.param("base_frame", base_frame_, std::string("base_link"));
    nh_private.param("local_map_topic", local_map_topic, std::string("test_local_map"));
    nh_private.param("local_map_resolution", resolution_, 0.05);
    nh_private.param("local_map_cells_per_side", width_, 400);
    nh_private.param("invalid_measurements_are_free_space", invalid_measurements_are_free_space_, false);
    nh_private.param("unseen_is_freespace", unseen_is_freespace_, true);
    nh_.param("use_scan_header_stamp_for_tfs", use_scan_header_stamp_for_tfs_, false);

    nh_private.param("shift_threshold", shift_threshold_, 1.0);
    nh_private.param("reliable_inf_range", reliable_inf_range_, 5.0);
    nh_.param("cluster_dist_euclid", cluster_dist_euclid_, 0.13);
    nh_.param("min_points_per_cluster", min_points_per_cluster_, 3);

    // Initialize map
    // All probabilities are held in log-space
    l0_ = logit(UNKNOWN);
    l_min_ = logit(MIN_PROB);
    l_max_ = logit(MAX_PROB);
    l_.resize(width_ * width_);
    for (int i = 0; i < width_; i++){
      for (int j = 0; j < width_; j++){
        if (unseen_is_freespace_)
          l_[i + width_ * j] = l_min_;
        else
          l_[i + width_ * j] = l0_;
      }
    }

	laserScanSubscriber_ = nh_.subscribe("/scan", 1, &OccupancyGridMapping::laserCallback, this);
    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(local_map_topic, 10);

	timer = nh_.createTimer(ros::Duration(0.05), &OccupancyGridMapping::reset, this);
  }

private:
	std::string scanTopic_;
	std::string fixed_frame_;
	std::string base_frame_;

	ros::NodeHandle nh_;

	ros::Subscriber laserScanSubscriber_;
	ros::Publisher map_pub_;

	ros::Time last_time_;
	ros::Time latest_scan_header_stamp_with_tf_available_;
	ros::Timer timer;

	std::vector<double> l_;

	double l0_;
	double l_min_;
	double l_max_;
	double resolution_;
	double grid_centre_pos_x_;
	double grid_centre_pos_y_;
	double shift_threshold_;
	double reliable_inf_range_;
	double cluster_dist_euclid_;

	int width_;
	int min_points_per_cluster_;

	bool grid_centre_pos_found_;
	bool invalid_measurements_are_free_space_;
	bool use_scan_header_stamp_for_tfs_;
	bool unseen_is_freespace_;

	tf::TransformListener tfl_;

	void reset(const ros::TimerEvent&){

		ROS_WARN("Reseting grid...");

		// Initialize map
		// All probabilities are held in log-space
		l0_ = logit(UNKNOWN);
		l_min_ = logit(MIN_PROB);
		l_max_ = logit(MAX_PROB);
		l_.resize(width_ * width_);
		for (int i = 0; i < width_; i++){
			for (int j = 0; j < width_; j++){
				if (unseen_is_freespace_)
					l_[i + width_ * j] = l_min_;
				else
					l_[i + width_ * j] = l0_;
			}
		}
	}

	/**
	* @brief Coordinated callback for both laser scan message
	*/
	void laserCallback(const sensor_msgs::LaserScan::ConstPtr &scanMsg) {
		
		// Find out the time that should be used for tfs
		bool transform_available;
		ros::Time tf_time;
		
		if (use_scan_header_stamp_for_tfs_) {
			// Use time from scan header
			tf_time = scanMsg->header.stamp;
			try {
				tfl_.waitForTransform(fixed_frame_, scanMsg->header.frame_id, tf_time, ros::Duration(1.0));
				transform_available = true;
			} catch (tf::TransformException ex) {
				ROS_INFO("Local map: No tf available");
				transform_available = false;
			}
		} else {
			// Otherwise just use the latest tf available
			tf_time = ros::Time(0);
			transform_available = tfl_.canTransform(fixed_frame_, scanMsg->header.frame_id, tf_time);
		}

		if (transform_available) {
			// Get the local grid occupancy map

			// Get the pose of the laser in the fixed frame
			bool transform_succesful;
			geometry_msgs::PoseStamped init_pose;
			geometry_msgs::PoseStamped laser_pose_fixed_frame;
			init_pose.header.frame_id = scanMsg->header.frame_id;
			init_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
			init_pose.header.stamp = tf_time;
			try {
				tfl_.transformPose(fixed_frame_, init_pose, laser_pose_fixed_frame);
				transform_succesful = true;
			} catch (tf::TransformException ex) {
				ROS_ERROR("Local map tf error: %s", ex.what());
				transform_succesful = false;
			}

			if (transform_succesful) {
				// Get the position of the laser
				double laser_x = laser_pose_fixed_frame.pose.position.x;
				double laser_y = laser_pose_fixed_frame.pose.position.y;
				double laser_yaw = tf::getYaw(laser_pose_fixed_frame.pose.orientation);

				// Get position of the local occupancy grid relative to the fixed frame
				if (grid_centre_pos_found_ == false) {
					grid_centre_pos_found_ = true;
					grid_centre_pos_x_ = laser_x;
					grid_centre_pos_y_ = laser_y;
				}

				// Check if we need to shift the local grid to be more centred on the
				// laser
				if (sqrt(pow(grid_centre_pos_x_ - laser_x, 2) + pow(grid_centre_pos_y_ - laser_y, 2)) > shift_threshold_) {
					// Shifting the local grid
					int translate_x = -(int)round((grid_centre_pos_x_ - laser_x) / resolution_);
					int translate_y = -(int)round((grid_centre_pos_y_ - laser_y) / resolution_);

					// Could translate in place to optimize later if needed
					std::vector<double> l_translated;
					l_translated.resize(width_ * width_);
					for (int i = 0; i < width_; i++) {
						for (int j = 0; j < width_; j++){
							int translated_i = i + translate_x;
							int translated_j = j + translate_y;
							if (translated_i >= 0 and translated_i < width_ and
								translated_j >= 0 and translated_j < width_) {
								l_translated[i + width_ * j] = l_[translated_i + width_ * translated_j];
							} else {
								if (unseen_is_freespace_)
									l_translated[i + width_ * j] = l_min_;
								else
									l_translated[i + width_ * j] = l0_;
							}
						}
					}

					l_ = l_translated;
					grid_centre_pos_x_ = laser_x;
					grid_centre_pos_y_ = laser_y;
        		}

				// Update the local occupancy grid with the new scan
				for (int i = 0; i < width_; i++) {
					for (int j = 0; j < width_; j++) {
						double m_update;

						// Find dist and angle of current cell to laser position
						double dist = sqrt(pow(i * resolution_ + grid_centre_pos_x_ - (width_ / 2.0) * resolution_ - laser_x, 2.0) +
										pow(j * resolution_ + grid_centre_pos_y_ - (width_ / 2.0) * resolution_ - laser_y, 2.0));
						double angle = betweenPIandNegPI( atan2(j * resolution_ + grid_centre_pos_y_ - (width_ / 2.0) * resolution_ - laser_y, 
										i * resolution_ + grid_centre_pos_x_ - (width_ / 2.0) * resolution_ - laser_x) - laser_yaw);
						bool is_human;

					if (angle > scanMsg->angle_min - scanMsg->angle_increment / 2.0 and angle < scanMsg->angle_max + scanMsg->angle_increment / 2.0) {
						// Find applicable laser measurement
						double closest_beam_angle = round(angle / scanMsg->angle_increment) * scanMsg->angle_increment;
						int closest_beam_idx = (int)round(angle / scanMsg->angle_increment) + scanMsg->ranges.size() / 2;

						// Processing the range value of the closest_beam to determine if
						// it's a valid measurement or not.
						// Sometimes it returns infs and NaNs that have to be dealt with
						bool valid_measurement;
						if (scanMsg->range_min <= scanMsg->ranges[closest_beam_idx] && scanMsg->ranges[closest_beam_idx] <= scanMsg->range_max) {
							// This is a valid measurement.
							valid_measurement = true;
						} else if (!std::isfinite(scanMsg->ranges[closest_beam_idx]) && scanMsg->ranges[closest_beam_idx] < 0) {
							// Object too close to measure.
							valid_measurement = false;
						} else if (!std::isfinite(scanMsg->ranges[closest_beam_idx]) && scanMsg->ranges[closest_beam_idx] > 0) {
							// No objects detected in range.
							valid_measurement = true;
						} else if (std::isnan(scanMsg->ranges[closest_beam_idx])) {
							// This is an erroneous, invalid, or missing measurement.
							valid_measurement = false;
						} else {
							// The sensor reported these measurements as valid, but they are
							// discarded per the limits defined by minimum_range and
							// maximum_range.
							valid_measurement = false;
						}

						if (valid_measurement) {
							double dist_rel = dist - scanMsg->ranges[closest_beam_idx];
							double angle_rel = angle - closest_beam_angle;
							if (dist > scanMsg->range_max or dist > scanMsg->ranges[closest_beam_idx] + ALPHA / 2.0 or
								fabs(angle_rel) > BETA / 2 or (!std::isfinite(scanMsg->ranges[closest_beam_idx]) and dist > reliable_inf_range_))
								m_update = UNKNOWN;
							else if (scanMsg->ranges[closest_beam_idx] < scanMsg->range_max and fabs(dist_rel) < ALPHA / 2 and !is_human)
								m_update = OBSTACLE;
							else
								m_update = FREE_SPACE;
						} else {
							// Assume cells corresponding to erroneous measurements are
							// either in freespace or unknown
							if (invalid_measurements_are_free_space_)
								m_update = FREE_SPACE;
							else
								m_update = UNKNOWN;
						}
					} else {
						m_update = UNKNOWN;
					}

					// update l_ using m_update
					l_[i + width_ * j] = (l_[i + width_ * j] + logit(m_update) - l0_);
					if (l_[i + width_ * j] < l_min_) l_[i + width_ * j] = l_min_;
					else if (l_[i + width_ * j] > l_max_) l_[i + width_ * j] = l_max_;
				}
			}

			// Create and fill out an OccupancyGrid message
			nav_msgs::OccupancyGrid m_msg;
			m_msg.header.stamp = scanMsg->header.stamp; // ros::Time::now();
			m_msg.header.frame_id = fixed_frame_;
			m_msg.info.resolution = resolution_;
			m_msg.info.width = width_;
			m_msg.info.height = width_;
			m_msg.info.origin.position.x = grid_centre_pos_x_ - (width_ / 2.0) * resolution_;
			m_msg.info.origin.position.y = grid_centre_pos_y_ - (width_ / 2.0) * resolution_;
			for (int i = 0; i < width_; i++)
				for (int j = 0; j < width_; j++)
					m_msg.data.push_back((int)(inverseLogit(l_[width_ * i + j]) * 100));

			// Publish!
			map_pub_.publish(m_msg);
			}
		}
	}

	/**
	* @basic The logit function, i.e., the inverse of the logstic function
	* @param p
	* @return The logit of p
	*/
	double logit(double p) { return log(p / (1 - p));}

	/**
	* @basic The inverse of the logit function, i.e., the logsitic function
	* @param p
	* @return The inverse logit of p
	*/
	double inverseLogit(double p) { return exp(p) / (1 + exp(p)); }

	/**
	* @basic Returns the equivalent of a passed-in angle in the -PI to PI range
	* @param angle_in The input angle
	* @return The angle in the range -PI to PI
	*/
	double betweenPIandNegPI(double angle_in) {
		double between_0_and_2PI = fmod(angle_in, 2 * M_PI);
		if (between_0_and_2PI < M_PI)
			return between_0_and_2PI;
		else
			return between_0_and_2PI - 2 * M_PI;
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "occupancy_grid_mapping");

	/** @todo We need to get a param, scan_topic, which is needed for the initialization
	list of OccupancyGridMapping. Is there a clearer way of doing this? */
	ros::NodeHandle nh;
	std::string scan_topic;
	nh.param("scan_topic", scan_topic, std::string("scan"));
	OccupancyGridMapping ogm(nh, scan_topic);

	ros::spin();
	return 0;
}
