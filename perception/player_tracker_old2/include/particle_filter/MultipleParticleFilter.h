#ifndef MULTIPLE_PARTICLE_FILTER_H_
#define MULTIPLE_PARTICLE_FILTER_H_

#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <player_tracker/Blob.h>
#include <player_tracker/PersonArray.h>

#include "visualization_msgs/Marker.h"


#include "particle_filter/Particle.h"
#include "particle_filter/ParticleFilter.h"

typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> AssociationMatrix;
typedef Eigen::Matrix<float, 1, Eigen::Dynamic> RowVector;
typedef Eigen::Matrix<float, Eigen::Dynamic, 1> ColVector;
typedef Eigen::Matrix<float, 3, 1> Vec3f;

typedef Eigen::Quaternionf Quaternion;

typedef std::set<int> IntSet;


class MultipleParticleFilter
{
private:
    int maxClouds;
    int numParticles;
    ParticleFilterList clouds;

    Vec3f robotPosition;
    Quaternion robotOrientation;

    ros::NodeHandle nh;
    ros::Publisher pubClouds;
    ros::Subscriber subPeopleDetector;
    ros::Subscriber subBlobDetector;
    ros::Subscriber poseSubscriber;

    ros::Publisher debugFov;

protected:
    virtual void initRosCommunication();

    virtual void scaleFov(Vec2f &left, Vec2f &right, float scale);

    virtual void track(int cloudIndex, const geometry_msgs::Pose &person);

    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped &pose);
    void getFovVertices(const player_tracker::Blob &person, Vec2f &robot, Vec2f &fovLeft, Vec2f &fovRight);

    virtual void setAssociation(const geometry_msgs::Pose &person, ColVector &association);
    virtual void setAssociation(const geometry_msgs::Point &person, ColVector &association);
    virtual int getMostAssociatedPerson(RowVector &association) const;
    virtual int getMostAssociatedCloud(ColVector &association) const;
    
    int cloneCloud(int cloudIndex);
    ParticleFilterPtr extractCloud(int cloudIndex);

    virtual void deleteCloudsIn(Vec2f &robot, Vec2f &left, Vec2f &right);
    virtual void deleteCloudsOutside(Vec2f &robot, Vec2f &left, Vec2f &right);
    virtual void deleteCloudsIf(std::function<bool (Vec2f &)> fovCondition);

    virtual bool isInFov(const Vec2f &point, const Vec2f &fovVertex1, const Vec2f &fovVertex2, const Vec2f &fovVertex3) const;
    virtual float sign(const Vec2f &p1, const Vec2f &p2, const Vec2f &p3) const;

    void deleteCloud(int index);
    void deleteClouds();
    void removeNullClouds();

    void publishClouds();

    virtual void computeAssociations(const player_tracker::PersonArray &people, AssociationMatrix &association, IntSet &peopleToAssociate);
    virtual void updateClouds(const player_tracker::PersonArray &people, AssociationMatrix &association, IntSet &peopleToAssociate);
    virtual void createNewClouds(const player_tracker::PersonArray &people, AssociationMatrix &association, IntSet &peopleToAssociate);

public:
    MultipleParticleFilter(int numParticles=500, int maxClouds=2);
    ~MultipleParticleFilter();

    void peopleDetectionCallback(const player_tracker::PersonArray &people);
    void blobDetectionCallback(const player_tracker::Blob &person);

};



#endif // MULTIPLE_PARTICLE_FILTER_H_
