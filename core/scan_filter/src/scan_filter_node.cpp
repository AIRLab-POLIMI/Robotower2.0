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


#include <boost/geometry/algorithms/centroid.hpp> 
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/assign.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/io/dsv/write.hpp>

typedef boost::geometry::model::d2::point_xy<double> point_xy;

class ScanFilter{
private:
    ros::Publisher scan_pub_;
    ros::NodeHandle nh_;
    std::string scan_topic_;
    std::string leg_array_topic_;
    std::string scan_out_topic_;
    std::vector<geometry_msgs::PointStamped> tower_centers_;
    geometry_msgs::Point playground_center_;    // wrt map

    tf::TransformListener listener_;


    float radius_;
    double MIN_ANGLE;


    boost::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> scan_sub_;
    boost::shared_ptr<message_filters::Subscriber<player_tracker::LegArray>> leg_clusters_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, player_tracker::LegArray> SynchronizerPolicy;
    boost::shared_ptr<message_filters::Synchronizer<SynchronizerPolicy>> sync_;



public:

    ScanFilter(std::string scan_topic, std::string leg_array_topic, std::string scan_out_topic, float radius): radius_(radius), 
        scan_topic_(scan_topic), scan_out_topic_(scan_out_topic), leg_array_topic_(leg_array_topic), 
        MIN_ANGLE(std::numeric_limits<double>::quiet_NaN())
     {

        scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>(scan_out_topic_, 10);
        scan_sub_ = boost::make_shared<message_filters::Subscriber<sensor_msgs::LaserScan>>(nh_, scan_topic_, 10);
        leg_clusters_sub_ = boost::make_shared<message_filters::Subscriber<player_tracker::LegArray>>(nh_, leg_array_topic, 10);
        //nh_.advertise<sensor_msgs::LaserScan>(scan_out_topic_.c_str(), 10);

        sync_ = boost::make_shared<message_filters::Synchronizer<SynchronizerPolicy>>(SynchronizerPolicy(10), *scan_sub_, *leg_clusters_sub_);
        sync_->registerCallback(boost::bind(&ScanFilter::callback, this, _1, _2));
        setTowerPosition();
        playground_center_ = getPlaygroundCenter();
    }

    geometry_msgs::Point getPlaygroundCenter(){
     
        // Create points to represent a 5x5 closed polygon.
        std::vector<point_xy> points;
        
        for (int i=0; i< tower_centers_.size(); i++){
            points.push_back(point_xy(tower_centers_[i].point.x, tower_centers_[i].point.y));
        }

        // Create a polygon object and assign the points to it.
        boost::geometry::model::polygon<point_xy> polygon;  
        boost::geometry::assign_points(polygon, points);

        point_xy c;
        boost::geometry::centroid(polygon,c);
        
        geometry_msgs::Point center;
        center.x = static_cast<float>(c.x());
        center.y = static_cast<float>(c.y());

        return center;

    }



    void setTowerPosition(){
        float num_tower;
        
        nh_.getParam("/num_towers", num_tower);


        for (int i=0; i < num_tower; i++){
            std::string str = "/tower_" + std::to_string(i+1);
            
            std::vector<float> tower_pos;
            if (!nh_.getParam(str, tower_pos)){
                ROS_FATAL_STREAM("Missing parameter: " << "/tower_" << std::to_string(i+1));
                exit(-1);
            }


            geometry_msgs::PointStamped pt;
            pt.point.x = tower_pos[0];
            pt.point.y = tower_pos[1];
            tower_centers_.push_back(pt);
        }
    }

    std::vector<geometry_msgs::PointStamped> towerPosFromMapToRobot(){
        std::vector<geometry_msgs::PointStamped> transformed_points;
        for(int i=0; i<tower_centers_.size(); i++){
            tower_centers_[i].header.stamp = ros::Time::now();
            geometry_msgs::PointStamped trans_point;

            listener_.transformPoint ("/base_link", tower_centers_[i], trans_point);
            transformed_points.push_back(trans_point); 
        }
        return transformed_points;
    }

    void callback(const sensor_msgs::LaserScanConstPtr& scan,  const player_tracker::LegArrayConstPtr& legs){

        // Get min angle from scan if undefined. Assumed constant.
        if(std::isnan(MIN_ANGLE)){
            MIN_ANGLE = scan->angle_min;
        }

        sensor_msgs::LaserScan filtered_scan(*scan);

        for(int i=0; i<legs->legs.size(); i++){

            std::vector<geometry_msgs::PointStamped> vect = towerPosFromMapToRobot();
            
            for(int t=0; t<vect.size(); t++){
                //Calculate distance
                float dist_from_tower = sqrt( pow(2, (vect[t].point.x - legs->legs[i].position.x)) + pow(2, (vect[t].point.y - legs->legs[i].position.y)) );
                
                if(dist_from_tower < radius_){
                    //Remove from scan if inside radius
                    
                    //calculate angle between robot and cluster
                    float theta = atan2(legs->legs[i].position.y, legs->legs[i].position.x);

                    //calculate scan vector position
                    int range_index = (theta - MIN_ANGLE) / scan->ranges.size();

                    filtered_scan.ranges[range_index] = std::numeric_limits<double>::quiet_NaN();
                    break;
                        
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



    ScanFilter filter("scan", "detected_leg_clusters", "scan_test", 15);

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
