#include <ros/ros.h>
#include <string>
#include <cmath>
#include <climits>
#include <cfloat>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <player_tracker/LegArray.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <tf/LinearMath/Vector3.h>

#include "player_tracker/laser_processor.h"

#include <boost/geometry/algorithms/centroid.hpp> 
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/assign.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/io/dsv/write.hpp>

typedef boost::geometry::model::d2::point_xy<double> point_xy;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, player_tracker::LegArray> SynchronizerPolicy;

class ScanFilter{
private:
    
    ros::Publisher scan_pub_;
    ros::Publisher markers_pub_;
    ros::Subscriber scan_sub_;
    ros::NodeHandle nh_;
    
    std::string fixed_frame_;
    std::string scan_topic_;
    std::string leg_array_topic_;
    std::string scan_out_topic_;
    
    std::vector<geometry_msgs::Point> tower_positions_;
    std::vector<tf::StampedTransform>  towers_robot_transforms_;

    geometry_msgs::Point playground_center_;    // wrt map
    tf::TransformListener* tf_listener_;

    int num_towers_;
    float radius_;
    double MIN_ANGLE;
    double cluster_dist_euclid_;
    int min_points_per_cluster_;
    bool use_scan_header_stamp_for_tfs_;

    tf::TransformListener tfl_;

    // boost::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> scan_sub_;
    // boost::shared_ptr<message_filters::Subscriber<player_tracker::LegArray>> leg_clusters_sub_;
    // boost::shared_ptr<message_filters::Synchronizer<SynchronizerPolicy>> sync_;

public:

    ScanFilter(std::string scan_topic, std::string leg_array_topic, std::string scan_out_topic, std::string fixed_frame, float radius): radius_(radius), 
        scan_topic_(scan_topic), fixed_frame_(fixed_frame), scan_out_topic_(scan_out_topic), leg_array_topic_(leg_array_topic), 
        MIN_ANGLE(std::numeric_limits<double>::quiet_NaN())
     {

        scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>(scan_out_topic_, 10);
        markers_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 20);
        
        scan_sub_ = nh_.subscribe(scan_topic_, 10, &ScanFilter::laserCallback, this);


        // scan_sub_ = boost::make_shared<message_filters::Subscriber<sensor_msgs::LaserScan>>(nh_, scan_topic_, 10);
        // leg_clusters_sub_ = boost::make_shared<message_filters::Subscriber<player_tracker::LegArray>>(nh_, leg_array_topic, 10);

        // sync_ = boost::make_shared<message_filters::Synchronizer<SynchronizerPolicy>>(SynchronizerPolicy(10), *scan_sub_, *leg_clusters_sub_);
        // sync_->registerCallback(boost::bind(&ScanFilter::callback, this, _1, _2));

        if (!nh_.getParam("/num_towers", num_towers_)){
            ROS_FATAL_STREAM("Missing parameter: /num_towers");
            exit(-1);
        }


        readTowerPosition();

        // playground_center_ = getPlaygroundCenter();
        tf_listener_ = new tf::TransformListener();
        towers_robot_transforms_.resize(num_towers_);

        nh_.param("cluster_dist_euclid", cluster_dist_euclid_, 0.13);
        nh_.param("min_points_per_cluster", min_points_per_cluster_, 3);
    }

    // geometry_msgs::Point getPlaygroundCenter(){
     
    //     // Create points to represent a 5x5 closed polygon.
    //     std::vector<point_xy> points;
        
    //     for (int i=0; i< tower_positions_.size(); i++){
    //         points.push_back(point_xy(tower_positions_[i].point.x, tower_positions_[i].point.y));
    //     }

    //     // Create a polygon object and assign the points to it.
    //     boost::geometry::model::polygon<point_xy> polygon;  
    //     boost::geometry::assign_points(polygon, points);

    //     point_xy c;
    //     boost::geometry::centroid(polygon,c);
        
    //     geometry_msgs::Point center;
    //     center.x = static_cast<float>(c.x());
    //     center.y = static_cast<float>(c.y());

    //     return center;

    // }


    ~ScanFilter(void){
        delete tf_listener_;
    }

    void publishTowerMarker(tf::Vector3 &position, int id){
        
        visualization_msgs::Marker m;
        m.header.stamp = ros::Time::now();
        m.header.frame_id = fixed_frame_;
        m.ns = "TOWERS";
        m.id = id + 66666;
        m.type = m.SPHERE;
        m.pose.position.x = position.getX();
        m.pose.position.y = position.getY();
        m.pose.position.z = 0.2;
        m.scale.x = radius_;
        m.scale.y = radius_;
        m.scale.z = 0;
        m.color.a = 0.7;
        m.color.r = 0;
        m.color.g = 0;
        m.color.b = 1;
        markers_pub_.publish(m);

    }

    void readTowerPosition(){
        int num_tower;
        
        if (!nh_.getParam("/num_towers", num_tower)){
                ROS_FATAL_STREAM("Missing parameter: /num_towers");
                exit(-1);
            }

        for (int i=0; i < num_tower; i++){
            std::string str = "/tower_" + std::to_string(i+1);
            
            std::vector<float> tower_pos;
            if (!nh_.getParam(str, tower_pos)){
                ROS_FATAL_STREAM("Missing parameter: " << "/tower_" << std::to_string(i+1));
                exit(-1);
            }


            geometry_msgs::Point pt;
            pt.x = tower_pos[0];
            pt.y = tower_pos[1];
            tower_positions_.push_back(pt);
        }
    }

    void towerPosFromMapToRobot(){
        auto now = ros::Time(0);
        for (int i=0; i < num_towers_; i++){
            std::string tower_frame = "/tower_" + std::to_string(i+1);
            tf_listener_->waitForTransform(tower_frame, now, "/base_link", now, "/map", ros::Duration(0.20));
            tf_listener_->lookupTransform("/base_link", tower_frame, now, towers_robot_transforms_[i]);

            publishTowerMarker(towers_robot_transforms_[i].getOrigin(), i+1);
        }
    }

    void callback(const sensor_msgs::LaserScanConstPtr& scan,  const player_tracker::LegArrayConstPtr& legs){

        // Get min angle from scan if undefined. Assumed constant.
        if(std::isnan(MIN_ANGLE)){
            MIN_ANGLE = scan->angle_min;
        }

        sensor_msgs::LaserScan filtered_scan(*scan);

        for(int i=0; i<legs->legs.size(); i++){

            // update tower position wrt robot.
            towerPosFromMapToRobot();
            
            for(int t=0; t<towers_robot_transforms_.size(); t++){
                //Calculate distance
                float dist_from_tower = std::sqrt( std::pow((towers_robot_transforms_[t].getOrigin().x() - legs->legs[i].position.x), 2) + 
                                                std::pow((towers_robot_transforms_[t].getOrigin().y() - legs->legs[i].position.y), 2));
                
                if(dist_from_tower < radius_){
                    ROS_DEBUG("# cluster member %u", legs->legs[i].point_indexes.size());
                    //Remove from scan if inside radius
                    for(int k=0; k<legs->legs[i].point_indexes.size(); k++){
                        int range_index = legs->legs[i].point_indexes[k];
                        ROS_DEBUG("Setting position %d to NaN around tower %d", range_index, t);
                        filtered_scan.ranges[range_index] = std::numeric_limits<double>::quiet_NaN();
                    }
                    
                    //break;      // exit from loop since cluster was matched to one of the towers.    
                }
            }
        }

        //Publishing filtered scan
        scan_pub_.publish(filtered_scan);
    }



    void laserCallback(const sensor_msgs::LaserScanConstPtr& scan){
        float max_range = 5.6;

        // Get min angle from scan if undefined. Assumed constant.
        if(std::isnan(MIN_ANGLE)){
            MIN_ANGLE = scan->angle_min;
        }

        towerPosFromMapToRobot();
        sensor_msgs::LaserScan filtered_scan(*scan);

        laser_processor::ScanProcessor processor(*scan); 
        processor.splitConnected(cluster_dist_euclid_);        
        processor.removeLessThan(min_points_per_cluster_);

        // Find out the time that should be used for tfs
        bool transform_available;
        ros::Time tf_time;
        // Use time from scan header
        if (use_scan_header_stamp_for_tfs_){
            tf_time = scan->header.stamp;

            try{
                tfl_.waitForTransform(fixed_frame_, scan->header.frame_id, tf_time, ros::Duration(1.0));
                transform_available = tfl_.canTransform(fixed_frame_, scan->header.frame_id, tf_time);
            }catch(tf::TransformException ex){
                ROS_INFO_STREAM("scan_filter node: No tf available at time: " << tf_time);
                transform_available = false;
            }
        }else{
            // Otherwise just use the latest tf available
            tf_time = ros::Time(0);
            transform_available = tfl_.canTransform(fixed_frame_, scan->header.frame_id, tf_time);
        }
        
        if (!transform_available){
            ROS_INFO("Not publishing /scan_filtered msg because no tf was available");
        }else{ // transform_available
            // Iterate through all clusters
            for (std::list<laser_processor::SampleSet*>::iterator cluster = processor.getClusters().begin();
                cluster != processor.getClusters().end();cluster++){   
            
                // Get position of cluster in map frame
                tf::Stamped<tf::Point> position((*cluster)->getPosition(), tf_time, scan->header.frame_id);
                
                for(int t=0; t<towers_robot_transforms_.size(); t++){
                    //Calculate distance
                    float dist_from_tower = std::sqrt(std::pow(towers_robot_transforms_[t].getOrigin().x() - position[0], 2) + 
                                                        std::pow(towers_robot_transforms_[t].getOrigin().y() - position[1], 2));

                    if(dist_from_tower < radius_){
                        //ROS_DEBUG("# cluster member %d", legs->legs[i].point_indexes.size());
                        //Remove from scan if inside radius
                        std::vector<int> point_indexes = (*cluster)->getSampleIndexes();
                        for(int k=0; k< point_indexes.size(); k++){
                            int range_index = point_indexes[k];
                            ROS_DEBUG("Setting position %d to NaN around tower %d", range_index, t);
                            filtered_scan.ranges[range_index] = max_range + 1; // This distance will be ignored by the costmap as above laser range
                            //PREVIOUS_VALUE std::numeric_limits<double>::quiet_NaN();
                        }
                    }
                }
            }
        }

        //Publishing filtered scan
        scan_pub_.publish(filtered_scan);
    }

};

int main (int argc, char** argv){
    // Initialize the ROS system and become a node .
    ros::init(argc , argv, "scan_filter");



    ScanFilter filter("scan", "detected_leg_clusters", "scan_filtered", "base_link", 0.50);

    // HeartbeatClient Initialize.
    // HeartbeatClient hb(tower_manager.nh, 0.2);
	// hb.start();
    // heartbeat::State::_value_type state = heartbeat::State::INIT;
    // hb.setState(state);

    // Loop at 10Hz until the node is shutdown.
    ros::Rate rate(10);
    
    // set heartbeat node state to started
    // state = heartbeat::State::STARTED;
    // bool success = hb.setState(state);
    
    ros::Duration(3.0).sleep(); // sleep for 3 seconds before beginning.

     while(ros::ok()){
        // Issue heartbeat.
        // hb.alive();
        // Wait until it's time for another iteration.
        rate.sleep();
        ros::spinOnce();
    }
    // success = hb.setState(heartbeat::State::STOPPED);
    // // Issue heartbeat.
    // hb.alive();
    // hb.stop();
}
