#ifndef PLAYER_TRACKER_MOTION_VARIANCE_H_
#define PLAYER_TRACKER_MOTION_VARIANCE_H_

#include <tf/transform_listener.h>

#include <targetted_human_tracker/motion_detector.h>
#include <targetted_human_tracker/Blob.h>
#include <targetted_human_tracker/TrackVariance.h>
#include <targetted_human_tracker/PersonArray.h>
#include <targetted_human_tracker/Person.h>

typedef struct MinimalPointCloud {
    float variance;
    Eigen::VectorXd centroid;
    sensor_msgs::PointCloud points;
} MinimalPointCloud;

typedef std::vector<MinimalPointCloud> MinimalPointCloudList;

#define OMP_THREADS 8

class MotionVariance : public Extractor {
public:
    MotionVariance(ros::NodeHandle nh, std::string scan_topic, float window_duration, std::string log_filename);
    MotionVariance(ros::NodeHandle nh, std::string scan_topic, float window_duration);
    ~MotionVariance();

protected:
    void initRosComunication();

    void convertClouds(Cloud2List &clusters, PointCloudList &clouds);

    virtual MinimalPointCloud minimizeCloud(sensor_msgs::PointCloud &pc, Eigen::MatrixXd cloudMatrix, Eigen::MatrixXd &projection);
    virtual float computeDistance(MinimalPointCloud &cloud, const geometry_msgs::Pose &pose);

    void publishVariance(float variance, sensor_msgs::PointCloud cloud);
    void publishVariance(MinimalPointCloud &minimalCloud);

private:
    void blobDetectionCallback(const targetted_human_tracker::Blob &msg);
    void trackCallback(const targetted_human_tracker::PersonArray &msg);
    void convertToOdomFrame(const targetted_human_tracker::PersonArray &msg);

    ros::Subscriber blob_sub_;
    ros::Publisher variance_pub_;
    ros::Subscriber player_track_sub_;

    std::vector<geometry_msgs::PoseStamped> track_poses_;
};

#endif // targetted_human_tracker_MOTION_VARIANCE_H_
