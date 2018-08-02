#include <particle_filter/BlobPublisher.h>

BlobPublisher::BlobPublisher(ros::NodeHandle &rosnode, std::string nodeName) : rosnode(rosnode)
{
    rosnode.param<std::string>(nodeName + "/player_frame", playerFrameName, "/player_link");
    rosnode.param<std::string>(nodeName + "/reference_frame", referenceFrameName, "/map");

    ROS_INFO("player_frame for blob_publisher:    %s", playerFrameName.c_str());
    ROS_INFO("reference_name for blob_publisher:  %s", referenceFrameName.c_str());

    std::string fovTopicName, blobTopicName;
    rosnode.param<std::string>(nodeName + "/fov_topic", fovTopicName, "/fov_marker");
    rosnode.param<std::string>(nodeName + "/blob_topic", blobTopicName, "/blob_detection");

    ROS_INFO("fov_topic for blob_publisher:       %s", fovTopicName.c_str());
    ROS_INFO("blob_topic for blob_publisher:      %s", blobTopicName.c_str());

    fovSub = rosnode.subscribe(fovTopicName, 1, &BlobPublisher::fovCallback, this);
    blobPub = rosnode.advertise<player_tracker::Blob>(blobTopicName, 10);
}

BlobPublisher::~BlobPublisher()
{ /*    */ }


void BlobPublisher::fovCallback(const visualization_msgs::Marker &fov)
{
    Vec2f robot, fov1, fov2;
    player_tracker::Blob msg;

    msg.observed = false;
    msg.id = 1;
    

    robot << fov.points[0].x, fov.points[0].y;
    fov1 << fov.points[1].x, fov.points[1].y;
    fov2 << fov.points[3].x, fov.points[3].y;

    msg.fov_sx.x = fov1.x();
    msg.fov_sx.y = fov1.y();
    msg.fov_dx.x = fov2.x();
    msg.fov_dx.y = fov2.y();

    try{
        msg.pose = getPlayerPose();
        msg.observed = isInFov(msg.pose, robot, fov1, fov2);
        blobPub.publish(msg);

    } catch (tf::TransformException ex) {
        ROS_WARN("%s",ex.what());
    }
}

geometry_msgs::Pose BlobPublisher::getPlayerPose()
{
    geometry_msgs::PoseStamped player;
    tf::StampedTransform transform;
    
    listener.waitForTransform(this->referenceFrameName, this->playerFrameName, ros::Time(0), ros::Duration(3.0));
    listener.lookupTransform(this->referenceFrameName, this->playerFrameName, ros::Time(0), transform);
    convert(transform, player);
    
    return player.pose;
}

bool BlobPublisher::isInFov(geometry_msgs::Pose &pose, Vec2f &robot, Vec2f &fov1, Vec2f &fov2)
{
    // TODO should consider smaller FOV (smaller angle)
    Vec2f point;
    point << pose.position.x, pose.position.y;

    return isPointInFov(point, robot, fov1, fov2);
}

bool BlobPublisher::isPointInFov(Vec2f &point, Vec2f &fovVertex1, Vec2f &fovVertex2, Vec2f &fovVertex3)
{
    bool b1 = sign(point, fovVertex1, fovVertex2) < 0.0f;
    bool b2 = sign(point, fovVertex2, fovVertex3) < 0.0f;
    bool b3 = sign(point, fovVertex3, fovVertex1) < 0.0f;

    return ((b1 == b2) && (b2 == b3));
}

float BlobPublisher::sign(Vec2f &p1, Vec2f &p2, Vec2f &p3)
{
    Vec2f diff1 = p1 - p3;
    Vec2f diff2 = p2 - p3;
    return diff1.x() * diff2.y() - diff2.x() * diff1.y();
}

void BlobPublisher::convert(tf::StampedTransform &transform, geometry_msgs::PoseStamped &player)
{
    player.pose.position.x = transform.getOrigin().getX();
    player.pose.position.y = transform.getOrigin().getY();
    player.pose.position.z = transform.getOrigin().getZ();

    player.pose.orientation.x = transform.getRotation().x();
    player.pose.orientation.y = transform.getRotation().y();
    player.pose.orientation.z = transform.getRotation().z();
    player.pose.orientation.w = transform.getRotation().w();
}
