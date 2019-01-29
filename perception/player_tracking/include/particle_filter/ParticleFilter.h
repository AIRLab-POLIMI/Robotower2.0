#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H

#include <random>
#include <vector>

#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <player_tracker/PersonArray.h>
#include <player_tracker/LegArray.h>
#include <geometry_msgs/PointStamped.h>
#include "particle_filter/Particle.h"
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>

#include <particle_filter/laser_processor.h>

#define WHITE_NOISE_MEAN 0.0f
#define WHITE_NOISE_STD 0.01f
#define PERTURBATION_PROB 0.01f

typedef struct{
    float mu;
    float variance;
}user_position;

class ParticleFilter {
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;
    
    int num_particles;                                      // number of particles
    ParticleList particles;
    float tao;                                              // temperature parameter used to compute the weights of the particles with kinect observations
    tf::TransformListener tf_listener;
    geometry_msgs::PointStamped last_estimate;

protected:
    virtual void initParticles();
    virtual void initParticles(const geometry_msgs::Pose &pose);
    virtual void propagateParticles();
    virtual void updateParticles(Eigen::Matrix<float, 3, 1> &Z);                                  // uses Kalman filter update equation
    virtual void updateParticles(Eigen::Matrix<float, 2, 1> &Z);
    virtual void computeWeights(Eigen::Matrix<float, 3, 1> &obs);
    virtual void computeWeights(Eigen::Matrix<float, 2, 1> &obs);
    virtual void computeKinectWeights(Eigen::Matrix<float, 3, 1> &obs);
    virtual void resample();
    virtual void resampleWithWheelMethod();
    virtual void deleteOldParticles();
    virtual void computeWeights();

    virtual int  getClosest(std::vector<std::tuple<geometry_msgs::PointStamped, double>> &vec,
                                geometry_msgs::Point state);
    virtual void publishParticles();
    virtual void deepCopy(const ParticleList &cloud);

    /*
     * Returns true for each person(observation) that must not be considered
     */
    bool blockObservation(const geometry_msgs::PointStamped &obs) const;
    std::vector<std::vector<int>> binomial_coefficient(int N, int K);

public:
    
    ParticleFilter(int num_particles=10);
    ParticleFilter(const ParticleFilter &that);
    ~ParticleFilter();

    virtual void callback(const sensor_msgs::LaserScanPtr &scan);
    virtual void track(const geometry_msgs::Pose &pose);
    virtual void track(const geometry_msgs::PointStamped &pose);
    virtual float computeAssociation(const geometry_msgs::Pose &person) const;
    virtual float computeAssociation(const geometry_msgs::Point &person) const;
    virtual void fillPoseArray(geometry_msgs::PoseArray &poses);
    void run();
    virtual Vec2f getCenter();
    geometry_msgs::PointStamped getOverallEstimate();
    geometry_msgs::PointStamped getOverallEstimateVariance();
    geometry_msgs::PointStamped getPlayerPose();
};

typedef ParticleFilter* ParticleFilterPtr;
typedef std::vector<ParticleFilterPtr> ParticleFilterList;



#endif //PARTICLEFILTER_H
