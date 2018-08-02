#include "particle_filter/MultipleParticleFilter.h"

MultipleParticleFilter::MultipleParticleFilter(int numParticles, int maxClouds) : numParticles(numParticles), maxClouds(maxClouds)
{
    // ParticleFilterPtr cloud = new ParticleFilter(numParticles);
    // clouds.push_back(cloud);

    initRosCommunication();

    robotPosition = Vec3f::Zero();
}

void MultipleParticleFilter::initRosCommunication()
{
    pubClouds = nh.advertise<geometry_msgs::PoseArray>("/tracking_clouds", 10);
    subPeopleDetector = nh.subscribe("/people_tracked", 10, &MultipleParticleFilter::peopleDetectionCallback, this);
    subBlobDetector = nh.subscribe("/blob_detection", 10, &MultipleParticleFilter::blobDetectionCallback, this);
    poseSubscriber = nh.subscribe("/amcl_pose", 1, &MultipleParticleFilter::poseCallback, this);

    debugFov = nh.advertise<visualization_msgs::Marker>("/debug", 2);
}

MultipleParticleFilter::~MultipleParticleFilter()
{
    deleteClouds();
}


void MultipleParticleFilter::deleteClouds()
{
    #pragma omp parallel for
    for (int i = 0; i < clouds.size(); i++) {
        deleteCloud(i);
    }
    clouds.clear();
}

void MultipleParticleFilter::deleteCloud(int index)
{
    delete clouds[index];
    clouds[index] = NULL;
}

void MultipleParticleFilter::removeNullClouds()
{
    for (int i = clouds.size() - 1; i >= 0; i--) {
        if (clouds[i] == NULL) {
            clouds.erase(clouds.begin() + i);
        }
    }
}

void MultipleParticleFilter::track(int cloudIndex, const geometry_msgs::Pose &person)
{
    clouds[cloudIndex]->track(person);
}

void MultipleParticleFilter::peopleDetectionCallback(const player_tracker::PersonArray &people)
{
    if (people.people.size() == 0) {
        return;
    }

    if (clouds.size() == 0) {
        clouds.push_back(new ParticleFilter(numParticles));
        AssociationMatrix association = AssociationMatrix::Zero(people.people.size(), clouds.size()); // #people X #clouds
        IntSet peopleToAssociate;
        computeAssociations(people, association, peopleToAssociate);
        updateClouds(people, association, peopleToAssociate);
    } else {
        AssociationMatrix association = AssociationMatrix::Zero(people.people.size(), clouds.size()); // #people X #clouds
        IntSet peopleToAssociate;
        computeAssociations(people, association, peopleToAssociate);
        updateClouds(people, association, peopleToAssociate);
        createNewClouds(people, association, peopleToAssociate);
    }
    // join overlapping clouds?

    publishClouds();
}

void MultipleParticleFilter::computeAssociations(const player_tracker::PersonArray &people, AssociationMatrix &association, IntSet &peopleToAssociate)
{
    #pragma omp parallel for
    for (int i = 0; i < people.people.size(); i++) {
        ColVector vec = RowVector::Zero(clouds.size());
        setAssociation(people.people[i].pose, vec);
        association.row(i) = vec;
        #pragma omp critical
        peopleToAssociate.insert(i);
    }
}

void MultipleParticleFilter::updateClouds(const player_tracker::PersonArray &people, AssociationMatrix &association, IntSet &peopleToAssociate)
{
    #pragma omp parallel for
    for (int i = 0; i < clouds.size(); i++) {
        RowVector vec = association.col(i);
        int j = getMostAssociatedPerson(vec);   // FIXME this must be disjoint-> each person must be unique, two people can not be associated to the same cloud
        if (j == -1) {
            continue;
        }
        track(i, people.people[j].pose);
        #pragma omp critical
        if (peopleToAssociate.find(j) != peopleToAssociate.end()) {
            peopleToAssociate.erase(j);
        }
    }
}

void MultipleParticleFilter::createNewClouds(const player_tracker::PersonArray &people, AssociationMatrix &association, IntSet &peopleToAssociate)
{
    // #pragma omp parallel for // cannot do this beacuse it is a set
    for (auto it = peopleToAssociate.begin(); it != peopleToAssociate.end(); ++it) {
        if (clouds.size() >= this->maxClouds) {
            continue;
        }
        ColVector vec = association.row(*it);
        int j = getMostAssociatedCloud(vec);
        ROS_INFO_STREAM( "cloud closer" << j << ", cloud size: " << clouds.size());
        if (j == -1) {
            continue;
        }
        int new_j = -1;
        #pragma omp critical
        new_j = cloneCloud(j);
        track(new_j, people.people[*it].pose);
    }
}

void MultipleParticleFilter::blobDetectionCallback(const player_tracker::Blob &person)
{
    // NOTE the blob detector is 100% reliable
    Vec2f robot, fovLeft, fovRight;
    getFovVertices(person, robot, fovLeft, fovRight);

    if (!person.observed) {
        // kill clouds in restricted fov
        scaleFov(fovLeft, fovRight, 0.25); // 0.25 to make it smaller and centered
        deleteCloudsIn(robot, fovLeft, fovRight);
    } else {

        // if do not have clouds create new one
        // update all clouds
        // kill clouds no in my enlarged fov

        if (clouds.size() == 0) {
            clouds.push_back(new ParticleFilter(numParticles));
        } else {
            scaleFov(fovLeft, fovRight, 1.25); // 1.25 to make it bigger and centered
            deleteCloudsOutside(robot, fovLeft, fovRight);
        }

        #pragma omp parallel for
        for (int i = 0; i < clouds.size(); i++) {
            track(i, person.pose);
        }
    }

    publishClouds();
}

void MultipleParticleFilter::getFovVertices(const player_tracker::Blob &person, Vec2f &robot, Vec2f &fovLeft, Vec2f &fovRight)
{
    Vec3f fullRobot = Vec3f::Zero();
    Vec3f fullFovLeft, fullFovRight;
    fullFovLeft << person.fov_sx.x, person.fov_sx.y, 1;
    fullFovRight << person.fov_dx.x, person.fov_dx.y, 1;

    Eigen::Matrix3f rotation = robotOrientation.normalized().toRotationMatrix();

    // FIXME the rotation matrix is 3x3 so also positions must be vector 3
    fullRobot = rotation * fullRobot + robotPosition;
    fullFovLeft = rotation * fullFovLeft + robotPosition;
    fullFovRight = rotation * fullFovRight + robotPosition;

    robot = fullRobot.topRows(2);
    fovLeft = fullFovLeft.topRows(2);
    fovRight = fullFovRight.topRows(2);
}

void MultipleParticleFilter::scaleFov(Vec2f &left, Vec2f &right, float scale)
{
    Vec2f vector = right - left;
    Vec2f versor = vector / vector.norm();
    float distance = vector.norm();

    right += distance * scale * versor;
    left -= distance * scale * versor;

    // debug printing points

    // visualization_msgs::Marker msg;
    // msg.type = 8;
    // msg.header.frame_id = "/map";
    // msg.lifetime = ros::Duration(0.1);
    // geometry_msgs::Point tmp;
    // tmp.x = left.x();
    // tmp.y = left.y();
    // geometry_msgs::Point tmp2;
    // tmp2.x = right.x();
    // tmp2.y = right.y();


    // msg.points.push_back(tmp);
    // msg.points.push_back(tmp2);
    // msg.color.a = 1;
    // msg.scale.x = 1.0;
    // msg.scale.y = 1.0;
    // debugFov.publish(msg);

}

void MultipleParticleFilter::deleteCloudsIn(Vec2f &robot, Vec2f &left, Vec2f &right)
{
    std::function<bool (Vec2f &)> fovCondition = [&, robot, left, right](Vec2f &center) { return isInFov(center, robot, left, right); };
    deleteCloudsIf(fovCondition);
    removeNullClouds();
}

void MultipleParticleFilter::deleteCloudsOutside(Vec2f &robot, Vec2f &left, Vec2f &right)
{
    std::function<bool (Vec2f &)> fovCondition = [&, robot, left, right](Vec2f &center) { return !isInFov(center, robot, left, right); };
    deleteCloudsIf(fovCondition);
    removeNullClouds();
}

void MultipleParticleFilter::deleteCloudsIf(std::function<bool (Vec2f &)> fovCondition)
{
    #pragma omp parallel for
    for (int i = 0; i < clouds.size(); i++) {
        Vec2f center = clouds[i]->getCenter();
        if (fovCondition(center)){
            deleteCloud(i);
        }
    }
}

bool MultipleParticleFilter::isInFov(const Vec2f &point, const Vec2f &fovVertex1, const Vec2f &fovVertex2, const Vec2f &fovVertex3) const
{
    bool b1 = sign(point, fovVertex1, fovVertex2) < 0.0f;
    bool b2 = sign(point, fovVertex2, fovVertex3) < 0.0f;
    bool b3 = sign(point, fovVertex3, fovVertex1) < 0.0f;

    return ((b1 == b2) && (b2 == b3));
}

float MultipleParticleFilter::sign(const Vec2f &p1, const Vec2f &p2, const Vec2f &p3) const
{
    Vec2f diff1 = p1 - p3;
    Vec2f diff2 = p2 - p3;
    return diff1.x() * diff2.y() - diff2.x() * diff1.y();
}

void MultipleParticleFilter::setAssociation(const geometry_msgs::Pose &person, ColVector &association)
{
    #pragma omp parallel for
    for (int i = 0; i < clouds.size(); i++) {
        association(i) = clouds[i]->computeAssociation(person);
    }
}

void MultipleParticleFilter::setAssociation(const geometry_msgs::Point &person, ColVector &association)
{
    #pragma omp parallel for
    for (int i = 0; i < clouds.size(); i++) {
        association(i) = clouds[i]->computeAssociation(person);
    }
}

// most associated person, wrt the cloud, is the person with the lowest value
int MultipleParticleFilter::getMostAssociatedPerson(RowVector &association) const
{
    // ROS_INFO_STREAM(association);
    if (association.sum() == 0) {
        return -1;
    }
    Eigen::MatrixXf::Index index;
    association.minCoeff(&index);
    return index;
}

// most associated cloud, wrt the cloud, is the cloud with the lowest value
int MultipleParticleFilter::getMostAssociatedCloud(ColVector &association) const
{
    if (association.sum() == 0) {
        return -1;
    }
    Eigen::MatrixXf::Index index;
    association.minCoeff(&index);
    return index;
}

int MultipleParticleFilter::cloneCloud(int cloudIndex)
{
    clouds.push_back(new ParticleFilter(*clouds[cloudIndex]));
    return clouds.size() - 1;
}

ParticleFilterPtr MultipleParticleFilter::extractCloud(int cloudIndex)
{
    ParticleFilterPtr cloud = clouds[cloudIndex];
    clouds.erase(clouds.begin() + cloudIndex);
    return cloud;
}

void MultipleParticleFilter::publishClouds()
{
    geometry_msgs::PoseArray cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "/map";

    #pragma omp parallel for
    for (int j = 0; j < clouds.size(); j++) {
        clouds[j]->fillPoseArray(cloud);
    }

    pubClouds.publish(cloud);
}

void MultipleParticleFilter::poseCallback(const geometry_msgs::PoseWithCovarianceStamped &pose)
{
    robotPosition.x() = pose.pose.pose.position.x;
    robotPosition.y() = pose.pose.pose.position.y;
    robotPosition.z() = pose.pose.pose.position.z;

    robotOrientation.x() = pose.pose.pose.orientation.x;
    robotOrientation.y() = pose.pose.pose.orientation.y;
    robotOrientation.z() = pose.pose.pose.orientation.z;
    robotOrientation.w() = pose.pose.pose.orientation.w;
}