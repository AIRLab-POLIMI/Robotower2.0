#ifndef PARTICLE_H
#define PARTICLE_H

#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include <eigen3/Eigen/Dense>

typedef Eigen::Matrix<float, 2, 1> Vec2f;
typedef Eigen::Matrix<float, 5, 1> Vec5f;
typedef Eigen::Matrix<float, 5, 5> Mat5f;
typedef Eigen::Matrix<float, 1, 3> RowVec3f;

class Particle{
private:
    float weight = 0;
    Vec5f state;
    Vec5f u;  // external motion (NO CONTROL INPUT ASSUMED) 
    Eigen::Matrix<float, 3, 5> H;  // measurement function 5 states - 3 observed (x,y, theta)
    Eigen::Matrix<float, 5, 5> I;  // identity matrix
    Eigen::Matrix<float, 3, 3> R;  // measurement uncertainty (3 uncorrelated measures with uncertainty)
    Eigen::Matrix<float, 5, 5> P;  // initial uncertainty
    Eigen::Matrix<float, 5, 5> F;  // Transition Matrix
    Eigen::Matrix<float, 5, 5> Q;  // process noise matrix


public:

    Particle(const Particle& that);
    Particle(Vec5f state);
    ~Particle();    

    /*
     * Propagate using Kalman Predict equation
     */
    void propagate();

    // void drift();

    float distance(Eigen::Matrix<float, 3, 1> &measure);
    float distance(Eigen::Matrix<float, 2, 1> &measure);

    void fillPose(geometry_msgs::Pose &pose);

    void setWeight(float weight);
    float getWeight();
    void normalizeWeight(float summation);
    Vec5f getState();

    /*
     * Implements standard Kalman filter update equation
     */
    void update(Eigen::Matrix<float, 3, 1> &Z);
    void update(Eigen::Matrix<float, 2, 1> &Z);

    Particle* perturbate(float noiseX, float noiseY, float noiseTheta=0.0f) const;

    // new functions
    bool isInsideConfidenceInterval(Vec2f &person);
    float confidenceLevel(Vec2f &person);
};

typedef Particle* ParticlePtr;
typedef std::vector<ParticlePtr> ParticleList;

#endif //PARTICLE_H
