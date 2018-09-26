#ifndef BLOB_PUBLISHER_H_
#define BLOB_PUBLISHER_H_

#include <iostream>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <player_tracker/Blob.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_listener.h>

#include <eigen3/Eigen/Dense>

typedef Eigen::Matrix<float, 3, 1> Vec3f;
typedef Eigen::Matrix<float, 2, 1> Vec2f;


class BlobPublisher {
private:
    ros::NodeHandle rosnode;
    ros::Subscriber fovSub;
    ros::Publisher blobPub;

    std::string playerFrameName;
    std::string referenceFrameName;

    tf::TransformListener listener;

protected:
    void fovCallback(const visualization_msgs::Marker &fov);
    
    virtual geometry_msgs::Pose getPlayerPose();
    
    virtual bool isInFov(geometry_msgs::Pose &pose, Vec2f &robot, Vec2f &fov1, Vec2f &fov2);
    virtual bool isPointInFov(Vec2f &point, Vec2f &fovVertex1, Vec2f &fovVertex2, Vec2f &fovVertex3);
    virtual float sign(Vec2f &p1, Vec2f &p2, Vec2f &p3);
    
    void convert(tf::StampedTransform &transform, geometry_msgs::PoseStamped &player);

public:
    BlobPublisher(ros::NodeHandle &rosnode, std::string nodeName);
    ~BlobPublisher();


};

#endif // BLOB_PUBLISHER_H_
