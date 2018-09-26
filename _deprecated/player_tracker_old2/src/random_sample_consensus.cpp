#include "player_tracker/random_sample_consensus.h"
#include <limits>
#include <geometry_msgs/Point32.h> 

#define WALL_MIN_SIZE 50

PlayerTracking::Ransac::Ransac(ros::NodeHandle nh, std::string base_frame): nh_(nh), base_frame_(base_frame){
    initRosComunication();
}

sensor_msgs::PointCloud PlayerTracking::Ransac::laserMsgAsPointCloud(const sensor_msgs::LaserScan::ConstPtr &msg){
    sensor_msgs::PointCloud cloud;
    projector_.transformLaserScanToPointCloud(base_frame_, *msg, cloud, tf_listener_);
    return cloud;
}


void PlayerTracking::Ransac::initRosComunication(){
    laser_scan_subscriber_    = nh_.subscribe("/scan", 1, &Ransac::laserCallback, this);
    // cloud_pub_  = nh_.advertise<sensor_msgs::PointCloud>("cloud_for_ransac", 10);
    obstacles_pub_  = nh_.advertise<sensor_msgs::LaserScan>("scan_obstacles", 10);
    inliers_pub_  = nh_.advertise<sensor_msgs::LaserScan>("scan_walls", 10);
}


void PlayerTracking::Ransac::pointcloud_to_laserscan(Eigen::MatrixXf points, sensor_msgs::LaserScanPtr output)
{

	uint32_t ranges_size = std::ceil((output->angle_max - output->angle_min) / output->angle_increment);
	output->ranges.assign(ranges_size, output->range_max + 1); // This distance will be ignored by the costmap as above laser range
	//std::numeric_limits<double>::infinity()); // + 1.0);

	for(int i=0; i<points.cols(); i++)
	{
		const float &x = points(0,i);
		const float &y = points(1,i);
		const float &z = points(2,i);

		if ( std::isnan(x) || std::isnan(y) || std::isnan(z) )
		{
			ROS_DEBUG("rejected for nan in point(%f, %f, %f)\n", x, y, z);
			continue;
		}

		double range_sq = y*y+x*x;
		double range_min_sq_ = output->range_min * output->range_min;
		if (range_sq < range_min_sq_) {
			ROS_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range_sq, range_min_sq_, x, y, z);
			continue;
		}

		double angle = atan2(y, x);
		if (angle < output->angle_min || angle > output->angle_max)
		{
			ROS_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output->angle_min, output->angle_max);
			continue;
		}
		int index = (angle - output->angle_min) / output->angle_increment;

/*
		if (output->ranges[index] * output->ranges[index] > range_sq)
		*/
		//Ignore if the detected range is largere than the max_range
		if(output->range_max * output->range_max > range_sq){
			output->ranges[index] = sqrt(range_sq);
		}
	}

	
}

void PlayerTracking::Ransac::laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg){

    // check whether we can transform data
    try{
        if (!tf_listener_.waitForTransform(msg->header.frame_id,
            base_frame_, msg->header.stamp + ros::Duration().fromSec(msg->ranges.size() *
            msg->time_increment), ros::Duration(1.0))) {
            return;
        }
    }catch (std::exception& ex){
        ROS_ERROR("%s", ex.what());
        return;
    }

    // convert laser scan into cloud
    sensor_msgs::PointCloud output_cloud = laserMsgAsPointCloud(msg);
   
    sensor_msgs::PointCloud wall_cloud;

    geometry_msgs::Point32 inf_pt;

    inf_pt.x = std::numeric_limits<double>::infinity();
    inf_pt.y = std::numeric_limits<double>::infinity();

    wall_cloud.points = std::vector<geometry_msgs::Point32>(output_cloud.points.size(), inf_pt);
    
    for (int i=0; i < 5; i++){

        //cloud_pub_.publish(cloud);

        sensor_msgs::PointCloud2 laser_as_pt_cloud_2;
        
        // convert PointCloud into PointCloud2
        int success = sensor_msgs::convertPointCloudToPointCloud2(output_cloud, laser_as_pt_cloud_2);

        if (!success){
            ROS_ERROR("error when converting PointCloud to PointCloud2!");
        }else{
            // Change from type sensor_msgs::PointCloud2 to pcl::PointXYZ
            pcl::PCLPointCloud2 to_convert;
            pcl_conversions::toPCL(laser_as_pt_cloud_2, to_convert);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_converted(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromPCLPointCloud2(to_convert,*cloud_converted);
            //////

            std::vector<int> inliers = runRansac(cloud_converted);
            ROS_DEBUG("INLIERS LENGHT %ul", inliers.size());
            for (int j=0; j < inliers.size() && inliers.size() > WALL_MIN_SIZE; j++){
                // saving walls
                wall_cloud.points[inliers[j]].x = output_cloud.points[inliers[j]].x;
                wall_cloud.points[inliers[j]].y = output_cloud.points[inliers[j]].y;
                
                // clearing inliers
                output_cloud.points[inliers[j]].x = std::numeric_limits<double>::infinity();
                output_cloud.points[inliers[j]].y = std::numeric_limits<double>::infinity();

            }
        }
    }

    /***** CONVERTING PointCloud to LaserScan and Publishing obstacles*****/
    sensor_msgs::PointCloud2 laser_as_pt_cloud_2;
     // convert PointCloud into PointCloud2
    int success = sensor_msgs::convertPointCloudToPointCloud2(output_cloud, laser_as_pt_cloud_2);
    if (!success){
        ROS_ERROR("error when converting PointCloud to PointCloud2!");
    }else{
        // Change from type sensor_msgs::PointCloud2 to pcl::PointXYZ
        pcl::PCLPointCloud2 to_convert;
        pcl_conversions::toPCL(laser_as_pt_cloud_2, to_convert);

        Eigen::MatrixXf points;
        pcl::getPointCloudAsEigen(to_convert,points);

        
        sensor_msgs::LaserScanPtr output_as_laser(new sensor_msgs::LaserScan());
        output_as_laser->header = msg->header;
        output_as_laser->angle_min = msg->angle_min;
        output_as_laser->angle_max = msg->angle_max;
        output_as_laser->angle_increment = msg->angle_increment;
        output_as_laser->time_increment = msg->time_increment;
        output_as_laser->scan_time = msg->scan_time;
        output_as_laser->range_min = msg->range_min;
        output_as_laser->range_max = msg->range_max;

        pointcloud_to_laserscan(points, output_as_laser);

        // publish data
        obstacles_pub_.publish(output_as_laser);
    }
    /*************************************************************************/

    /***** CONVERTING PointCloud to LaserScan and Publishing wall_obstacles*****/
    // convert PointCloud into PointCloud2
    success = sensor_msgs::convertPointCloudToPointCloud2(wall_cloud, laser_as_pt_cloud_2);
    if (!success){
        ROS_ERROR("error when converting PointCloud to PointCloud2!");
    }else{

        /***** CONVERTING PointCloud to LaserScan and Publishing obstacles*****/
        // Change from type sensor_msgs::PointCloud2 to pcl::PointXYZ
        pcl::PCLPointCloud2 to_convert;
        pcl_conversions::toPCL(laser_as_pt_cloud_2, to_convert);

        Eigen::MatrixXf points;
        pcl::getPointCloudAsEigen(to_convert,points);

        
        sensor_msgs::LaserScanPtr output_as_laser(new sensor_msgs::LaserScan());
        output_as_laser->header = msg->header;
        output_as_laser->angle_min = msg->angle_min;
        output_as_laser->angle_max = msg->angle_max;
        output_as_laser->angle_increment = msg->angle_increment;
        output_as_laser->time_increment = msg->time_increment;
        output_as_laser->scan_time = msg->scan_time;
        output_as_laser->range_min = msg->range_min;
        output_as_laser->range_max = msg->range_max;

        pointcloud_to_laserscan(points, output_as_laser);

        // publish data
        inliers_pub_.publish(output_as_laser);
    }
    /*************************************************************************/

}

std::vector<int> PlayerTracking::Ransac::runRansac(pcl::PointCloud<pcl::PointXYZ>::Ptr in){

    // initialize output
    pcl::PointCloud<pcl::PointXYZ>::Ptr result_from_ransac (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> inliers;

    // created RandomSampleConsensus object and compute the appropriated model (linear)
    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr
        model_p (new pcl::SampleConsensusModelLine<pcl::PointXYZ> (in));
    
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
    ransac.setDistanceThreshold (.1);
    ransac.computeModel();
    ransac.getInliers(inliers);
    
    return inliers;
}


/////////////////////////////* MAIN *//////////////////////////////////

int main(int argc, char** argv){
    ros::init(argc, argv, "ransac");
    ros::NodeHandle nh;
    PlayerTracking::Ransac ransac(nh, "base_link");
    ros::spin();
    return 0;
}
