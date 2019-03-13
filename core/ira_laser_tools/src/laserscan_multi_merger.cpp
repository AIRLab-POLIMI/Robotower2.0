#include <ros/ros.h>
#include <string.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h> 
#include "sensor_msgs/LaserScan.h"
#include "pcl_ros/point_cloud.h"
#include <Eigen/Dense>
#include <dynamic_reconfigure/server.h>
#include <ira_laser_tools/laserscan_multi_mergerConfig.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;
using namespace pcl;
using namespace laserscan_multi_merger;

class LaserscanMerger
{

typedef message_filters::Subscriber<sensor_msgs::LaserScan> LaserSubscriber;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> sync_policy;

public:
    LaserscanMerger();
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan, std::string topic);
	void syncCallback(const sensor_msgs::LaserScan::ConstPtr& scan_1, const sensor_msgs::LaserScan::ConstPtr& scan_2);
    void pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLPointCloud2 *merged_cloud);
    void reconfigureCallback(laserscan_multi_mergerConfig &config, uint32_t level);

private:
    ros::NodeHandle node_;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener tfListener_;

    ros::Publisher point_cloud_publisher_;
    ros::Publisher laser_scan_publisher_;
    vector<ros::Subscriber> scan_subscribers;
    vector<bool> clouds_modified;

    vector<pcl::PCLPointCloud2> clouds;
    vector<string> input_topics;

	vector<double> times;   //< TODO

    void laserscan_topic_parser();

    double angle_min;
    double angle_max;
    double angle_increment;
    double time_increment;
    double scan_time;
    double range_min;
    double range_max;

    string destination_frame;
    string cloud_destination_topic;
    string scan_destination_topic;
    string laserscan_topics;

};

void LaserscanMerger::reconfigureCallback(laserscan_multi_mergerConfig &config, uint32_t level)
{
	this->angle_min = config.angle_min;
	this->angle_max = config.angle_max;
	this->angle_increment = config.angle_increment;
	this->time_increment = config.time_increment;
	this->scan_time = config.scan_time;
	this->range_min = config.range_min;
	this->range_max = config.range_max;
}

void LaserscanMerger::laserscan_topic_parser()
{
	// LaserScan topics to subscribe
	ros::master::V_TopicInfo topics;
	ros::master::getTopics(topics);

    istringstream iss(laserscan_topics);
	vector<string> tokens;
	copy(istream_iterator<string>(iss), istream_iterator<string>(), back_inserter<vector<string> >(tokens));

	vector<string> tmp_input_topics;

	for(int i=0;i<tokens.size();++i)
	{
	        for(int j=0;j<topics.size();++j)
		{
			if( (tokens[i].compare(topics[j].name) == 0) && (topics[j].datatype.compare("sensor_msgs/LaserScan") == 0) )
			{
				tmp_input_topics.push_back(topics[j].name);
			}
		}
	}

	sort(tmp_input_topics.begin(),tmp_input_topics.end());
	std::vector<string>::iterator last = std::unique(tmp_input_topics.begin(), tmp_input_topics.end());
	tmp_input_topics.erase(last, tmp_input_topics.end());


	// Do not re-subscribe if the topics are the same
	if( (tmp_input_topics.size() != input_topics.size()) || !equal(tmp_input_topics.begin(),tmp_input_topics.end(),input_topics.begin()))
	{

		// Unsubscribe from previous topics
		for(int i=0; i<scan_subscribers.size(); ++i)
			scan_subscribers[i].shutdown();

		input_topics = tmp_input_topics;
		if(input_topics.size() > 0)
		{
            scan_subscribers.resize(input_topics.size());
			clouds_modified.resize(input_topics.size());
			clouds.resize(input_topics.size());
            ROS_INFO("Subscribing to topics\t%ld", scan_subscribers.size());
			for(int i=0; i<input_topics.size(); ++i)
			{
                scan_subscribers[i] = node_.subscribe<sensor_msgs::LaserScan> (input_topics[i].c_str(), 10, boost::bind(&LaserscanMerger::scanCallback,this, _1, input_topics[i]));
				clouds_modified[i] = false;
				cout << input_topics[i] << " ";
			}
		}
		else
            ROS_INFO("Not subscribed to any topic.");
	}
}

LaserscanMerger::LaserscanMerger(): times(2)
{
	ros::NodeHandle nh("~");

	nh.getParam("destination_frame", destination_frame);
	nh.getParam("cloud_destination_topic", cloud_destination_topic);
	nh.getParam("scan_destination_topic", scan_destination_topic);
    nh.getParam("laserscan_topics", laserscan_topics);

    nh.param("angle_min", angle_min, -2.36);
    nh.param("angle_max", angle_max, 2.36);
    nh.param("angle_increment", angle_increment, 0.0058);
    nh.param("scan_time", scan_time, 0.0333333);
    nh.param("range_min", range_min, 0.45);
    nh.param("range_max", range_max, 25.0);

    this->laserscan_topic_parser();

	point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> (cloud_destination_topic.c_str(), 1, false);
	laser_scan_publisher_ = node_.advertise<sensor_msgs::LaserScan> (scan_destination_topic.c_str(), 1, false);
	
	LaserscanMerger::LaserSubscriber right_sub(node_, "/scan_right", 1);
	LaserscanMerger::LaserSubscriber left_sub(node_, "/scan_left", 1);


	//The real queue size for synchronisation is set here.
    message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> MySyncPolicy(10);
    MySyncPolicy.setAgePenalty(1000); //set high age penalty to publish older data faster even if it might not be correctly synchronized.


    // Create synchronization policy. Here: async because time stamps will never match exactly
    const message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> MyConstSyncPolicy = MySyncPolicy;
    message_filters::Synchronizer< message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> > sync(MyConstSyncPolicy,
                                                                                       right_sub,
                                                                                       left_sub);
    // Register one callback for all topics
    sync.registerCallback(boost::bind(&LaserscanMerger::syncCallback, this, _1, _2));

	// message_filters::Synchronizer<LaserscanMerger::sync_policy> class_sync(sync_policy(10), right_sub, left_sub);
	// class_sync.registerCallback(boost::bind(&LaserscanMerger::syncCallback, this, _1, _2));
}

void LaserscanMerger::syncCallback(const sensor_msgs::LaserScan::ConstPtr& scan_1, const sensor_msgs::LaserScan::ConstPtr& scan_2){
	ROS_INFO("WEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE");
	sensor_msgs::PointCloud tmpCloud1,tmpCloud2;
	sensor_msgs::PointCloud2 tmpCloud3;

	sensor_msgs::PointCloud tmpCloud4,tmpCloud5;
	sensor_msgs::PointCloud2 tmpCloud6;

    // Verify that TF knows how to transform from the received scan to the destination scan frame
	tfListener_.waitForTransform(scan_1->header.frame_id.c_str(), destination_frame.c_str(), ros::Time(0), ros::Duration(1));// scan->header.stamp, ros::Duration(1));
    projector_.transformLaserScanToPointCloud(scan_1->header.frame_id, *scan_1, tmpCloud1, tfListener_, laser_geometry::channel_option::Distance);

	tfListener_.waitForTransform(scan_2->header.frame_id.c_str(), destination_frame.c_str(), ros::Time(0), ros::Duration(1));// scan->header.stamp, ros::Duration(1));
    projector_.transformLaserScanToPointCloud(scan_2->header.frame_id, *scan_2, tmpCloud4, tfListener_, laser_geometry::channel_option::Distance);
	try
	{
		tfListener_.transformPointCloud(destination_frame.c_str(), tmpCloud1, tmpCloud2);
		tfListener_.transformPointCloud(destination_frame.c_str(), tmpCloud4, tmpCloud5);
	}catch (tf::TransformException ex){ROS_ERROR("%s",ex.what());return;}

	sensor_msgs::convertPointCloudToPointCloud2(tmpCloud2,tmpCloud3);
	pcl_conversions::toPCL(tmpCloud3, clouds[0]);
	clouds_modified[0] = true;

	sensor_msgs::convertPointCloudToPointCloud2(tmpCloud5,tmpCloud6);
	pcl_conversions::toPCL(tmpCloud6, clouds[1]);
	clouds_modified[1] = true;

	

	// for(int i=0; i<input_topics.size(); ++i)
	// {
	// 	if(topic.compare(input_topics[i]) == 0)
	// 	{
	// 		sensor_msgs::convertPointCloudToPointCloud2(tmpCloud2,tmpCloud3);
	// 		pcl_conversions::toPCL(tmpCloud3, clouds[i]);
	// 		clouds_modified[i] = true;
	// 	}
	// }	


	pcl::PCLPointCloud2 merged_cloud = clouds[0];
	pcl::concatenatePointCloud(merged_cloud, clouds[1], merged_cloud);
	point_cloud_publisher_.publish(merged_cloud);

	Eigen::MatrixXf points;
	getPointCloudAsEigen(merged_cloud,points);

	pointcloud_to_laserscan(points, &merged_cloud);

    /*// Count how many scans we have
	int totalClouds = 0;
	for(int i=0; i<clouds_modified.size(); ++i)
		if(clouds_modified[i])
			++totalClouds;

    // Go ahead only if all subscribed scans have arrived
	if(totalClouds == clouds_modified.size())
	{
		pcl::PCLPointCloud2 merged_cloud = clouds[0];
		clouds_modified[0] = false;

		for(int i=1; i<clouds_modified.size(); ++i)
		{
			pcl::concatenatePointCloud(merged_cloud, clouds[i], merged_cloud);
			clouds_modified[i] = false;
		}
	
		point_cloud_publisher_.publish(merged_cloud);

		Eigen::MatrixXf points;
		getPointCloudAsEigen(merged_cloud,points);

		pointcloud_to_laserscan(points, &merged_cloud);
	}*/
}


void LaserscanMerger::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan, std::string topic)
{
	sensor_msgs::PointCloud tmpCloud1,tmpCloud2;
	sensor_msgs::PointCloud2 tmpCloud3;

    // Verify that TF knows how to transform from the received scan to the destination scan frame
	tfListener_.waitForTransform(scan->header.frame_id.c_str(), destination_frame.c_str(), ros::Time(0), ros::Duration(1));// scan->header.stamp, ros::Duration(1));
    projector_.transformLaserScanToPointCloud(scan->header.frame_id, *scan, tmpCloud1, tfListener_, laser_geometry::channel_option::Distance);
	try
	{
		tfListener_.transformPointCloud(destination_frame.c_str(), tmpCloud1, tmpCloud2);
	}catch (tf::TransformException ex){ROS_ERROR("%s",ex.what());return;}

	for(int i=0; i<input_topics.size(); ++i)
	{
		if(topic.compare(input_topics[i]) == 0)
		{
			times[i] = tmpCloud2.header.stamp.toSec();
			sensor_msgs::convertPointCloudToPointCloud2(tmpCloud2,tmpCloud3);
			pcl_conversions::toPCL(tmpCloud3, clouds[i]);
			clouds_modified[i] = true;
		}
	}	

    // Count how many scans we have
	int totalClouds = 0;
	for(int i=0; i<clouds_modified.size(); ++i)
		if(clouds_modified[i])
			++totalClouds;

    // Go ahead only if all subscribed scans have arrived
	if(totalClouds == clouds_modified.size())
	{
		double diff = fabs(times[1] - times[0]);
		long double now =  ros::Time::now().toSec(); 
		// ROS_WARN_STREAM("LaserScan time diff: " << diff);
		// ROS_WARN_STREAM("\tCurrent ROS time: " <<now);

		pcl::PCLPointCloud2 merged_cloud = clouds[0];
		clouds_modified[0] = false;

		for(int i=1; i<clouds_modified.size(); ++i)
		{
			pcl::concatenatePointCloud(merged_cloud, clouds[i], merged_cloud);
			clouds_modified[i] = false;
		}
	
		point_cloud_publisher_.publish(merged_cloud);

		Eigen::MatrixXf points;
		getPointCloudAsEigen(merged_cloud,points);

		pointcloud_to_laserscan(points, &merged_cloud);
	}
	
	// else{
	// 	ROS_ERROR("Discarded msg..");
	// }
}

void LaserscanMerger::pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLPointCloud2 *merged_cloud)
{
	sensor_msgs::LaserScanPtr output(new sensor_msgs::LaserScan());
	output->header = pcl_conversions::fromPCL(merged_cloud->header);
	output->header.frame_id = destination_frame.c_str();
	output->header.stamp = ros::Time::now();  //fixes #265
	output->angle_min = this->angle_min;
	output->angle_max = this->angle_max;
	output->angle_increment = this->angle_increment;
	output->time_increment = this->time_increment;
	output->scan_time = this->scan_time;
	output->range_min = this->range_min;
	output->range_max = this->range_max;

	uint32_t ranges_size = std::ceil((output->angle_max - output->angle_min) / output->angle_increment);
	output->ranges.assign(ranges_size, output->range_max + 1.0);

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


		if (output->ranges[index] * output->ranges[index] > range_sq)
			output->ranges[index] = sqrt(range_sq);
	}

	laser_scan_publisher_.publish(output);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "laser_multi_merger");

    LaserscanMerger _laser_merger;

    dynamic_reconfigure::Server<laserscan_multi_mergerConfig> server;
    dynamic_reconfigure::Server<laserscan_multi_mergerConfig>::CallbackType f;

    f = boost::bind(&LaserscanMerger::reconfigureCallback,&_laser_merger, _1, _2);
	server.setCallback(f);

	ros::spin();

	return 0;
}
