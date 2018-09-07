#ifndef KF_FUSION_H

/* ROS related includes */
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int16.h>
#include <player_tracker/LegArray.h>
#include <player_tracker/PersonEvidenceArray.h>
#include <player_tracker/PersonArray.h>


#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#include <visualization_msgs/Marker.h>

/* Eigen */
#include <eigen3/Eigen/Dense>

#include <boost/geometry/algorithms/centroid.hpp> 
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/assign.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/io/dsv/write.hpp>

#include <vector>

#define RATE 0.1

#define EMPTY -1000

// typedef message_filters::sync_policies::ApproximateTime<std_msgs::Int16, player_tracker::LegArray> SynchronizerPolicy;

class KF_fusion{
private:
    Eigen::Matrix<float, 4, 1> x;  // initial state (location and velocity)
    Eigen::Matrix<float, 4, 1> u;  // external motion (NO CONTROL INPUT ASSUMED) 
    Eigen::Matrix<float, 2, 4> H;  // measurement function 4 states - 2 observed (angle and distance)
    Eigen::Matrix<float, 4, 4> I;  // identity matrix
    Eigen::Matrix<float, 2, 2> R;  // measurement uncertainty (2 uncorrelated measures with uncertainty)
    Eigen::Matrix<float, 4, 4> P;  // initial uncertainty
    Eigen::Matrix<float, 4, 4> F;  // Transition Matrix
    Eigen::Matrix<float, 4, 4> Q;  // process noise matrix

    ros::NodeHandle nh_;

    // boost::shared_ptr<message_filters::Subscriber<std_msgs::Int16>> angle_sub_;
    // boost::shared_ptr<message_filters::Subscriber<player_tracker::LegArray>> leg_clusters_sub_;
    // boost::shared_ptr<message_filters::Synchronizer<SynchronizerPolicy>> sync_;
    ros::Subscriber angle_sub_;
    ros::Subscriber leg_clusters_sub_;
    ros::Publisher pub_;
    ros::Publisher vis_pub;

    
    
    void reduced_KFF(float angle, bool laser_camera);
    void angleCallback(const std_msgs::Int16ConstPtr& angle);
    void legCallback(const player_tracker::PersonArrayConstPtr& player_evidence);
    
    std_msgs::Int16 last_angle;
    player_tracker::LegArray last_leg;

public:
    KF_fusion();
    void kalmanUpdate(float angle_camera, float angle_laser , float distance_laser ); //, float distance_player);
    void kalmanPredict();
    void publishKalman(int data_index, float distance);
    double getDistancePlayer(const std::vector<geometry_msgs::Point>& centroids_person, const std::vector<geometry_msgs::Point>& detected_legs, float origin_x, float origin_y);

};

#endif

KF_fusion::KF_fusion(){
    float delta_t = (1./RATE);

    // Kalman filter matrices
    x << 0,0,0,0;
    u << 0,0,0,0;
    H << 1,0,0,0,   // measurement function 6 states - 2 observed (angle and distance)
         1,0,0,0; //,
         //0,1,0,0; //,
        //  0,1,0,0;  

    I << 1,0,0,0,            // identity matrix
         0,1,0,0,
         0,0,1,0,
         0,0,0,1;     

    R << 0.2,0, //0,  0,                       // measurement uncertainty (2 uncorrelated measures with uncertainty)
         0,5; //0, //0,
        // 0,0,0.8; //,0,
        //  0,0,0,10;

    P << 1,0,0.01,0,             // initial uncertainty
         0,1,0,0.01,
         0.01,0,1,0,
         0,0.01,0,1;      
    
    // Transition Matrix
    F << 1,0,delta_t,0,         // angle
         0,1,0,delta_t,         // distance (polar coordinates)
         0,0,1,0,               // angular speed
         0,0,0,1;               // velocity 

    Q << 10,0,0.01,0,         // process noise matrix
         0,10,0,0.01,
         0.01,0,0.01,0,
         0,0.01,0,0.01;

    angle_sub_ = nh_.subscribe("arduino/current_servo_angle", 1, &KF_fusion::angleCallback, this);
    pub_ = nh_.advertise<geometry_msgs::PoseStamped>("kf_fusion_data_player_pos", 1);
    vis_pub = nh_.advertise<visualization_msgs::Marker>( "kf_fusion_marker", 0 );
}

void KF_fusion::publishKalman(int data_index, float distance){
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = "base_link";
    msg.header.stamp = ros::Time::now();
    msg.pose.position.x = cos(x(0,0))*distance; // x(1,0);
    msg.pose.position.y = sin(x(0,0))*distance; //x(1,0);
    
    pub_.publish(msg);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.ns = "kf_fusion";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.pose.position.x = msg.pose.position.x;
    marker.pose.position.y = msg.pose.position.y;
    marker.pose.position.z = 0.05;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    if (data_index == -1){
        //Update only with laser
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
    }
    else if(data_index == 0){
        //Update only with camera
         marker.color.r = 0.0;
         marker.color.g = 0.0;
         marker.color.b = 1.0;
    }
    else{
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
    }
    vis_pub.publish( marker );

}

void KF_fusion::kalmanPredict(){
    x = F * x + u;
    P = F*P*F.transpose() + Q;
}

void KF_fusion::angleCallback(const std_msgs::Int16ConstPtr& angle){
    last_angle = *angle;
}

void KF_fusion::legCallback(const player_tracker::PersonArrayConstPtr& player_evidence){
  
}

void KF_fusion::reduced_KFF(float angle, bool laser_camera){
    Eigen::Matrix<float, 1, 1> Z;
    Eigen::Matrix<float, 1, 4> H;
    Eigen::Matrix<float, 1, 1> R;

     Z << angle;
     H << 1,0,0,0;
     R << 0.6;

     auto Y =  Z.transpose() - (H*x);
     auto S = ((H*P) * H.transpose()) + R;
     auto K = (P * H.transpose()) * S.inverse();
     x = x + (K * Y);
     P = (I - (K*H)) * P;
}

void KF_fusion::kalmanUpdate(float angle_camera, float angle_laser, float distance_laser){ //, float distance_person_evidence){
    if ((angle_camera == EMPTY)||(angle_laser == EMPTY)){
        // Case only Laser has info
        reduced_KFF( angle_camera == EMPTY ? angle_laser : angle_camera, angle_camera == EMPTY ? true : false );
    }
    else{
        Eigen::Matrix<float, 1, 2> Z;
        Z << angle_camera, angle_laser; //, distance_laser; // , 1, distance_person_evidence;

        auto Y =  Z.transpose() - (H*x);
        auto S = ((H*P) * H.transpose()) + R;
        auto K = (P * H.transpose()) * S.inverse();
        x = x + (K * Y);
        P = (I - (K*H)) * P;
    }
}

double KF_fusion::getDistancePlayer(const std::vector<geometry_msgs::Point>& centroids_person, const std::vector<geometry_msgs::Point>& detected_legs, float origin_x, float origin_y){
    // float min_distance, index_min_distance;
    // if (!centroids_person.empty()){
    //     min_distance = sqrt(pow(centroids_person[0].x - cos(x(0,0))*x(1,0),2) + pow(centroids_person[0].y - sin(x(0,0))*x(1,0),2));
    //     index_min_distance = 0;
    //     for(int i=1; i < centroids_person.size(); i++){
    //         if (sqrt(pow(centroids_person[i].x - cos(x(0,0))*x(1,0),2) + pow(centroids_person[i].y - sin(x(0,0))*x(1,0),2)) < min_distance){
    //             min_distance = sqrt(pow(centroids_person[i].x - cos(x(0,0))*x(1,0),2) + pow(centroids_person[i].y - sin(x(0,0))*x(1,0),2));
    //             index_min_distance = i;
    //         }
    //     }

    //     if (min_distance < 0.4)
    //         return sqrt(pow(centroids_person[index_min_distance].x - origin_x,2) + pow(centroids_person[index_min_distance].y - origin_y,2));     
    // }
    // if (!detected_legs.empty()){
    //     min_distance = sqrt(pow(detected_legs[0].x - cos(x(0,0))*x(1,0),2) + pow(detected_legs[0].y - sin(x(0,0))*x(1,0),2));
    //     index_min_distance = 0;
    //     for(int i=1; i < detected_legs.size(); i++){
    //         if (sqrt(pow(detected_legs[i].x - cos(x(0,0))*x(1,0),2) + pow(detected_legs[i].y - sin(x(0,0))*x(1,0),2)) < min_distance){
    //             min_distance = sqrt(pow(detected_legs[i].x - cos(x(0,0))*x(1,0),2) + pow(detected_legs[i].y - sin(x(0,0))*x(1,0),2));
    //             index_min_distance = i;
    //         }
    //         }
    //     return sqrt(pow(detected_legs[index_min_distance].x - origin_x,2) + pow(detected_legs[index_min_distance].y - origin_y,2));
    // }
    // else{
    //     ROS_INFO("Bothh data is not available or useful");
    //     return 1;
    // }

    double min_distance, index_min_distance;
    if (!centroids_person.empty()){
        min_distance = fabs(x(0,0) - atan2(centroids_person[0].y, centroids_person[0].x));
        index_min_distance = 0;
        for(int i=1; i < centroids_person.size(); i++){
            if (fabs(x(0,0) - atan2(centroids_person[i].y, centroids_person[i].x))){
                min_distance = fabs(x(0,0) - atan2(centroids_person[i].y, centroids_person[i].x));
                index_min_distance = i;
            }
        }

        if (min_distance < 10)
            return sqrt(pow(centroids_person[index_min_distance].x - origin_x,2) + pow(centroids_person[index_min_distance].y - origin_y,2));     
    }
    if (!detected_legs.empty()){
        min_distance =  fabs(x(0,0) - atan2(detected_legs[0].y, detected_legs[0].x));
        index_min_distance = 0;
        for(int i=1; i < detected_legs.size(); i++){
            if ( fabs(x(0,0) - atan2(detected_legs[i].y, detected_legs[i].x)) < min_distance){
                min_distance =  fabs(x(0,0) - atan2(detected_legs[i].y, detected_legs[i].x));
                index_min_distance = i;
                }
            }
        if(min_distance<10)
            return sqrt(pow(detected_legs[index_min_distance].x - origin_x,2) + pow(detected_legs[index_min_distance].y - origin_y,2));
    }
    else{
        ROS_INFO("Bothh data is not available or useful");
        return 1;
    }
        
}



std::vector<geometry_msgs::Point> centroids_person;
std::vector<geometry_msgs::Point> detected_legs;
tf::TransformListener *tf_listener_;
void legArrayCallback(const player_tracker::LegArray& legs ){
    std::vector<geometry_msgs::Point> tmp;
    geometry_msgs::PointStamped pt;
    for (int i=0; i < legs.legs.size(); i++){
        pt.header = legs.header;
        pt.point.x = legs.legs[i].position.x;
        pt.point.y = legs.legs[i].position.y;
        tf_listener_->transformPoint("base_link",pt,pt);
        tmp.push_back(pt.point);
    }
    detected_legs = tmp;
}

void peopleTrackedCallback(const player_tracker::PersonArray& people){
    std::vector<geometry_msgs::Point> tmp;
    geometry_msgs::PointStamped pt;
    for (int i=0; i < people.people.size(); i++){
        pt.header = people.header;
        pt.point.x = people.people[i].pose.position.x;
        pt.point.y = people.people[i].pose.position.y;
        tf_listener_->transformPoint("base_link",pt,pt);
        tmp.push_back(pt.point);
    }
    centroids_person = tmp;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "KF_fusion");
    ros::NodeHandle nh;
    ros::Subscriber leg_array_ = nh.subscribe("/detected_leg_clusters", 1, legArrayCallback);
    ros::Subscriber person_evidence_array_ = nh.subscribe("/people_tracked", 1, peopleTrackedCallback);

    std::string robot_base_ = "base_link";
    std::string player_base_ = "player_filtered_link";
    std::string camera_base_ = "cam_target_link";

    tf::StampedTransform transform_laser;
    tf::StampedTransform transform_cam;
    tf::StampedTransform transform_base_link;
    tf_listener_ = new tf::TransformListener;

    bool do_laser_update = false;
    bool do_cam_update = false;
    int data_index = -2;

    double distance_person_evidence, distance = 0;
    double last_angle_camera = 0;

    KF_fusion kf_fuse_filter;


    //   if (!nh.getParam("image_topic_name", image_topic_name)){
    //     ROS_FATAL("camera_reader: Could not read image_topic_name from rosparam server!");
    //     exit(-1);
    //   }
    ros::Rate loop_rate(10);
    while (nh.ok()) {
        
        // predict
        kf_fuse_filter.kalmanPredict();

        // get player link
        try{
            tf_listener_->waitForTransform(robot_base_.c_str(), ros::Time(0), player_base_.c_str(), ros::Time(0), "/map", ros::Duration(0.10));
            tf_listener_->lookupTransform(robot_base_.c_str(), player_base_.c_str(), ros::Time(0), transform_laser);
            tf_listener_->lookupTransform(robot_base_.c_str(), "/map", ros::Time(0), transform_base_link);
            do_laser_update = true;
        }
        catch (const std::exception& ex){
            ROS_WARN("KF_fusion: %s does not exist! Laser therefore goes on dead-reckoning.\n Details: %s",
            player_base_.c_str(), ex.what());
        }

        // Get camera link
        try{
            tf_listener_->waitForTransform(robot_base_.c_str(), ros::Time(0), camera_base_.c_str(), ros::Time::now(), "/map", ros::Duration(0.10));
            tf_listener_->lookupTransform(robot_base_.c_str(), camera_base_.c_str(), ros::Time(0), transform_cam);
            double last_angle = atan2(transform_cam.getOrigin().y(), transform_cam.getOrigin().x());
            if(last_angle != last_angle_camera)
                do_cam_update = true;
            else 
                do_cam_update = false;
        }
        catch (const std::exception& ex){
            ROS_WARN("KF_fusion: %s does not exist! Camera therefore goes on dead-reckoning.\n Details: %s",
            camera_base_.c_str(), ex.what());
        }

        if (do_laser_update && do_cam_update){
            // Update with both laser and camera data
            data_index = 1;
            double last_angle = atan2(transform_cam.getOrigin().y(), transform_cam.getOrigin().x());
            last_angle_camera = last_angle;
            double player_filtered_angle = atan2(transform_laser.getOrigin().y(), transform_laser.getOrigin().x());
           
            // distance = sqrt(pow(transform_laser.getOrigin().y(),2) + pow(transform_laser.getOrigin().x(),2));

            // distance_person_evidence = kf_fuse_filter.getDistancePlayer(centroids_person, detected_legs, transform_base_link.getOrigin().x(), transform_base_link.getOrigin().y());

            kf_fuse_filter.kalmanUpdate(last_angle, player_filtered_angle, distance ); // , distance_person_evidence);
            
            distance_person_evidence = kf_fuse_filter.getDistancePlayer(centroids_person, detected_legs, 0,0);
            
            if(fabs(distance - distance_person_evidence) <= 1.5)
                distance = distance_person_evidence;
            do_cam_update = false;
            do_laser_update = false;
            detected_legs.clear();
            centroids_person.clear();

        }else if (do_cam_update){
            // Update only with camera
            data_index = 0;
            double last_angle = atan2(transform_cam.getOrigin().y(), transform_cam.getOrigin().x());
            kf_fuse_filter.kalmanUpdate(last_angle, EMPTY, 0 ); //, 0);
            do_cam_update = false;
        }else if (do_laser_update){
            //Update with laser
            data_index = -1;
            double player_filtered_angle = atan2(transform_laser.getOrigin().y(), transform_laser.getOrigin().x());
            kf_fuse_filter.kalmanUpdate(EMPTY, player_filtered_angle, 0 ); //, 0);
            do_laser_update = false;
        }
   
        ROS_INFO_STREAM("Distance: " << distance);
        kf_fuse_filter.publishKalman(data_index, distance);

        ros::spinOnce();
        loop_rate.sleep();
    }
}
