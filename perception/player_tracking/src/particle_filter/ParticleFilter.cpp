#include "particle_filter/ParticleFilter.h"
#include <math.h>
#include <vector>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <random>
#include <map>
#include <player_tracker/LegProbability.h>
#include <player_tracker/Leg.h>
#include <geometry_msgs/PointStamped.h>
#include <tuple> 
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <boost/cast.hpp>
#include <algorithm>    // std::next_permutation


float rand_FloatRange(float a, float b){
    return ((b - a) * ((float)rand() / RAND_MAX)) + a;
}

ParticleFilter::ParticleFilter(int num_particles) : num_particles(num_particles) {
    particles.resize(num_particles);
    pub = nh.advertise<geometry_msgs::PoseArray>("/pf_cloud", 10);
    sub = nh.subscribe("scan_player_tracking", 10, &ParticleFilter::callback, this);
    initParticles();
}

ParticleFilter::ParticleFilter(const ParticleFilter &that){
    this->num_particles = that.num_particles;
    this->tao = that.tao;
    deepCopy(that.particles);

    pub = nh.advertise<geometry_msgs::PoseArray>("/pf_cloud", 10);
}

ParticleFilter::~ParticleFilter() {
    deleteOldParticles();
};

void ParticleFilter::initParticles(const geometry_msgs::Pose &pose){
    #pragma omp parallel for    
    for (int p=0; p < particles.size();p++){
        Eigen::Matrix<float, 5, 1> state;
        state << pose.position.x, pose.position.y, rand_FloatRange(0, 2*M_PI), 0, 0;
        particles[p] = new Particle(state);
    }
}

void ParticleFilter::initParticles() {  // particles init at random
    #pragma omp parallel for    
    for (int p=0; p < particles.size();p++) {
        Eigen::Matrix<float, 5, 1> state;
        // assumed -10 lowest x, 10 highest x possible (field of robogame)
        state << rand_FloatRange(-4, 4), rand_FloatRange(-4, 4), rand_FloatRange(0, 2*M_PI), 0, 0;  // TODO fix ranges of field
        particles[p] = new Particle(state);
    }
}

void ParticleFilter::propagateParticles() {
    #pragma omp parallel for
    for (int i = 0; i < particles.size(); i++) {
        particles[i]->propagate();
    }
}

void ParticleFilter::updateParticles(Eigen::Matrix<float, 3, 1> &Z) {
    #pragma omp parallel for
    for (int i = 0; i < particles.size(); i++) {
        particles[i]->update(Z);
    }
}

void ParticleFilter::updateParticles(Eigen::Matrix<float, 2, 1> &Z) {
    #pragma omp parallel for
    for (int i = 0; i < particles.size(); i++) {
        particles[i]->update(Z);
    }
}

geometry_msgs::PointStamped ParticleFilter::getOverallEstimate(){
    geometry_msgs::PointStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "base_link";
    auto center = getCenter();
    pose.point.x = center(0);
    pose.point.y = center(1);
    return pose;
}

geometry_msgs::PointStamped ParticleFilter::getOverallEstimateVariance(){
    double var_x = 0;
    double mean_x = last_estimate.point.x;
    for (int i = 0; i < particles.size(); i++) {
        Vec5f state = particles[i]->getState();
        var_x += (state(0) - mean_x) * (state(0) - mean_x);
    }
    var_x /= particles.size();
    // sd_x = sqrt(var_x);

    double var_y = 0;
    double mean_y = last_estimate.point.y;
    for (int i = 0; i < particles.size(); i++) {
        Vec5f state = particles[i]->getState();
        var_y += (state(0) - mean_y) * (state(0) - mean_y);
    }
    var_y /= particles.size();
    // sd_y = sqrt(var_y);

    
    geometry_msgs::PointStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "base_link";
    // auto center = getCenter();
    pose.point.x = var_x;
    pose.point.y = var_y;
    return pose;
}

void ParticleFilter::computeWeights(Eigen::Matrix<float, 2, 1> &obs) {
    float summation = 0.0f;
    
    #pragma omp parallel for reduction(+:summation)
    for (int i = 0; i < particles.size(); i++) {
        particles[i]->setWeight(1/particles[i]->distance(obs));
        summation += particles[i]->getWeight();
    }

    #pragma omp parallel for
    for (int i = 0; i < particles.size(); i++) {
        particles[i]->normalizeWeight(summation);
    }
}

// void ParticleFilter::computeWeights(Eigen::Matrix<float, 3, 1> &obs) {
//     float summation = 0.0f;

//     #pragma omp parallel for reduction(+:summation)
//     for (int i = 0; i < particles.size(); i++) {
//         particles[i]->setWeight(1 / particles[i]->distance(obs));
//         summation += particles[i]->getWeight();
//     }

//     #pragma omp parallel for
//     for (int i = 0; i < particles.size(); i++) {
//         particles[i]->normalizeWeight(summation);
//     }
// }

void ParticleFilter::computeWeights() {
    float summation = 0.0f;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<float> gauss(WHITE_NOISE_MEAN, WHITE_NOISE_STD);

    #pragma omp parallel for reduction(+:summation)
    for (int i = 0; i < particles.size(); i++) {
        particles[i]->setWeight(1);
        summation += particles[i]->getWeight();
    }

    #pragma omp parallel for
    for (int i = 0; i < particles.size(); i++) {
        float before = particles[i]->getWeight();
        particles[i]->normalizeWeight(summation);
        if (particles[i]->getWeight() < 0){
            ROS_FATAL("ComputeWeights(): BEFORE: %f \t SUM: %f \t NORM: %f", before, summation, particles[i]->getWeight());
            exit(-1);
        }
    }
}

void ParticleFilter::computeKinectWeights(Eigen::Matrix<float, 3, 1> &obs) {
    float summation = 0.0f;

    #pragma omp parallel for reduction(+:summation)
    for (int i = 0; i < particles.size(); i++) {

        float angle_diff = fabs(atan2(obs(1),(obs(0))) - obs(2));
        //ROS_INFO("Angle difference:  %f", angle_diff);
        particles[i]->setWeight(std::exp(- particles[i]->distance(obs) / this->tao) * std::exp(-angle_diff)); // numerator of Boltzmann
        particles[i]->setWeight(1 / particles[i]->distance(obs) + (1/angle_diff));
        summation += particles[i]->getWeight();  // denom of Boltzmann weights
    }

    #pragma omp parallel for
    for (int i = 0; i < particles.size(); i++) {
        particles[i]->normalizeWeight(summation);    // normalization of weights computed with Boltzmann
    }
}

void ParticleFilter::computeWeights(Eigen::Matrix<float, 3, 1> &obs) {
    float summation = 0.0f;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<float> gauss(WHITE_NOISE_MEAN, WHITE_NOISE_STD);

    #pragma omp parallel for reduction(+:summation)
    for (int i = 0; i < particles.size(); i++) {
        float new_weight = (1/particles[i]->distance(obs));
        particles[i]->setWeight(new_weight);
        summation += particles[i]->getWeight();  // denom of Boltzmann weights
    }

    #pragma omp parallel for
    for (int i = 0; i < particles.size(); i++) {
        float before = particles[i]->getWeight();
        particles[i]->normalizeWeight(summation);    // normalization of weights computed with Boltzmann
        if (particles[i]->getWeight() < 0){
            ROS_ERROR("On computeWeights(Eigen::Matrix<float, 3, 1> &obs): BEFORE: %f \t SUM: %f \t NORM: %f", before, summation, particles[i]->getWeight());
            exit(-1);
        }
    }
}

void ParticleFilter::resampleWithWheelMethod(){

    std::normal_distribution<float> gauss(WHITE_NOISE_MEAN, WHITE_NOISE_STD);

    std::default_random_engine generator;
    std::uniform_real_distribution<double> uniform_real_sampler(0.0,1.0);

    int N = particles.size();
    ParticleList newParticles;
    int index = uniform_real_sampler(generator) * N;
    float beta = 0.0;

    float max_weight = particles[0]->getWeight();

    // getting max particle weight
    for (int i=1; i < particles.size(); i++){
        if (particles[i]->getWeight() < 0){
            ROS_ERROR("Particle weight cannot be negative!");
            ros::shutdown();
            exit(-1);
        }

        if (particles[i]->getWeight() > max_weight){
            max_weight = particles[i]->getWeight();
        }
    }

    //ROS_DEBUG("Max weight: %f", max_weight);

    for (int i=0; i < N; i++){

        //ROS_INFO_STREAM("Loop: " << i << "\tsample: " << uniform_real_sampler(generator));
        beta += uniform_real_sampler(generator) * 2.0 * max_weight;
        while (beta > particles[index]->getWeight()){
            //ROS_INFO("beta= %f, index = %d, weight = %f", beta, index, particles[index]->getWeight());
            beta -= particles[index]->getWeight();
            index = (index + 1) % N;
        }

        // decide whether to perturbate particles
        if (uniform_real_sampler(generator) <= PERTURBATION_PROB){
            // perturbation
            ParticlePtr perturbated = particles[index]->perturbate(gauss(generator), gauss(generator));
            newParticles.push_back(perturbated);
        }else{
            // no perturbation
            newParticles.push_back(new Particle(*particles[index]));
        }

    }

    deleteOldParticles();
    particles = newParticles;

}


void ParticleFilter::resample()
{
    ParticleList newParticles;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<float> gauss(WHITE_NOISE_MEAN, WHITE_NOISE_STD);
    
    for (int i = 0; i < particles.size(); i++) {
        int qt = round(((float)particles.size()) * particles[i]->getWeight());
        for (int j = 0; j < qt; j++) {
            ParticlePtr perturbated = particles[i]->perturbate(gauss(gen), gauss(gen));
            newParticles.push_back(perturbated);
        }
        if (newParticles.size() >= num_particles) {
            break;
        }
    }

    // std::cout << "num particles: " << newParticles.size() << std::endl;
    deleteOldParticles();

    particles = newParticles;
}

void ParticleFilter::track(const geometry_msgs::PointStamped &pose){
    Eigen::Matrix<float, 3, 1> obs;

    tf::StampedTransform transform_cam;

    try{
        tf_listener.waitForTransform("base_link", ros::Time(0), "cam_target_link", ros::Time(0), "/map", ros::Duration(0.10));
        tf_listener.lookupTransform("base_link", "cam_target_link", ros::Time(0), transform_cam);
    }
    catch (const std::exception& ex){
        ROS_WARN("KF_fusion: %s does not exist! Camera therefore goes on dead-reckoning.\n Details: %s",
        "cam_target_link", ex.what());
    }

    double last_angle = atan2(transform_cam.getOrigin().y(), transform_cam.getOrigin().x());
    
    obs << pose.point.x, pose.point.y, last_angle;
    
    propagateParticles();
    updateParticles(obs);
    computeWeights(obs);
    resampleWithWheelMethod();
    publishParticles();
    last_estimate = getOverallEstimate();
}

void ParticleFilter::run(){
    propagateParticles();
    computeWeights();
    resampleWithWheelMethod();
    publishParticles();
    last_estimate = getOverallEstimate();
    // missing update of velocity and theta!
}

geometry_msgs::PointStamped ParticleFilter::getPlayerPose(){
    return last_estimate;
}

void ParticleFilter::track(const geometry_msgs::Pose &pose) {
    
    Eigen::Matrix<float, 3, 1> obs;
    obs << pose.position.x, pose.position.y, pose.orientation.z;

    //propagateParticles();
    updateParticles(obs);
    computeWeights(obs);
    resampleWithWheelMethod();
    publishParticles();
    last_estimate = getOverallEstimate();
}

void ParticleFilter::deepCopy(const ParticleList &cloud)
{
    for (int i = 0; i < cloud.size(); i++) {
        this->particles.push_back(new Particle(*cloud[i]));
    }
}

void ParticleFilter::deleteOldParticles() {
    #pragma omp parallel for
    for (int i = 0; i < particles.size(); i++) {
        delete particles[i];
    }
}

void ParticleFilter::publishParticles() {
    geometry_msgs::PoseArray cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "/base_link";

    #pragma omp parallel for
    for (int i = 0; i < particles.size(); i++) {
        geometry_msgs::Pose pose;
        particles[i]->fillPose(pose);
        #pragma omp critical
        cloud.poses.push_back(pose);
    }

    pub.publish(cloud);
}

void ParticleFilter::fillPoseArray(geometry_msgs::PoseArray &poses)
{
    #pragma omp parallel for
    for (int i = 0; i < particles.size(); i++) {
        geometry_msgs::Pose pose;
        particles[i]->fillPose(pose);
        #pragma omp critical
        poses.poses.push_back(pose);
    }
}

int ParticleFilter::getClosest(std::vector<std::tuple<geometry_msgs::PointStamped, double>> &vec,
                                geometry_msgs::Point state){
    int idx = 0;
    float dist = sqrt(pow(std::get<0>(vec[0]).point.x - state.x, 2) + pow(std::get<0>(vec[0]).point.y - state.y, 2));
    for(int i=1; i < vec.size(); i++){
        float new_dist = sqrt(pow(std::get<0>(vec[i]).point.x - state.x, 2) + pow(std::get<0>(vec[i]).point.y - state.y, 2));
        if ( new_dist < dist){
            idx = i;
            dist = new_dist;
        }
    }
    return idx;
}


std::vector<std::vector<int>> ParticleFilter::binomial_coefficient(int N, int K){
    std::string bitmask(K, 1); // K leading 1's
    bitmask.resize(N, 0); // N-K trailing 0's

    std::vector<std::vector<int>> permutations;

    // print integers and permute bitmask
    do {
        std::vector<int> perm;
        for (int i = 0; i < N; ++i){ // [0..N-1] integers
            if (bitmask[i]) perm.push_back(i);
        }
        permutations.push_back(perm);
    } while (std::prev_permutation(bitmask.begin(), bitmask.end()));
    return permutations;
}


void ParticleFilter::callback(const sensor_msgs::LaserScanPtr &scan){
    /* This should import people evidence... using a fallback to normal clusters in case no pair of legs are
    detected. Take care for allowing new tracking hypotheses to be born. Outside gating zone.*/


    double cluster_dist_euclid_ = 0.13;
    double min_points_per_cluster_ = 10;

    laser_processor::ScanProcessor processor(*scan); 
    processor.splitConnected(cluster_dist_euclid_);        
    processor.removeLessThan(min_points_per_cluster_);

    // Iterate through all clusters
    for (std::list<laser_processor::SampleSet*>::iterator cluster = processor.getClusters().begin();
         cluster != processor.getClusters().end(); cluster++){

        // Get position of cluster in laser frame
        // tf::Stamped<tf::Point> position((*cluster)->getPosition(), tf_time, scan->header.frame_id);
        // float rel_dist = pow(position[0]*position[0] + position[1]*position[1], 1./2.);

        tf::Point cluster_center = (*cluster)->getPosition();

        geometry_msgs::Pose pt;
        pt.position.x = cluster_center[0];
        pt.position.y = cluster_center[1];
        track(pt);

    }   
    // TODO: change the reference frame for base_link instead of map. THa 

    // do we have at least 2 legs?
    // if (msg->legs.size() >= 2){

    //     std::vector<std::tuple<geometry_msgs::PointStamped, double>> human_evidences;

    //     std::vector<std::vector<int>> perms = binomial_coefficient(msg->legs.size(),2);

    //     float prob = -1000;
    //     geometry_msgs::PointStamped best_pt; 

    //     int count_perm = 0;

    //     geometry_msgs::PoseArray cloud;
    //     cloud.header = msg->header;
    //     cloud.header.frame_id = "map";

    //     for (int i=0; i<perms.size(); i++){
    //         int item1 = perms[i][0];
    //         int item2 = perms[i][1];
        
    //         double distance = sqrt(pow(msg->legs[item1].position.x - msg->legs[item2].position.x, 2.0) + 
    //                             pow(msg->legs[item1].position.y - msg->legs[item2].position.y, 2.0));

    //         player_tracker::LegProbability srv;
    //         srv.request.distance = distance;

    //         if (leg_context_service_client.call(srv)){
    //             //ROS_INFO_STREAM("Probability: " << srv.response.probability);
    //             geometry_msgs::PointStamped pt;
    //             pt.header = msg->header;
    //             pt.point.x = (msg->legs[item1].position.x + msg->legs[item2].position.x) / 2;
    //             pt.point.y = (msg->legs[item1].position.y + msg->legs[item2].position.y) / 2;

    //             //tf_listener.transformPoint("base_link", pt, pt);

    //             if (srv.response.probability > prob){
    //                 best_pt = pt;
    //                 prob = srv.response.probability;
    //             }

    //             std::tuple<geometry_msgs::PointStamped, double> tuple_data;
    //             tuple_data = std::make_tuple(pt, distance);
    //             human_evidences.push_back(tuple_data);

    //             if ((srv.response.probability > 0.5)){
    //                 // TODO add gating here to avoid update everybody!
    //                 float time_dist_to_last = (pt.header.stamp.toSec() - last_estimate.header.stamp.toSec());

    //                 float dist_to_last = sqrt(pow(pt.point.x - last_estimate.point.x, 2) + 
    //                                         pow(pt.point.y - last_estimate.point.y, 2));

    //                 if (!(distance > 1) && !(time_dist_to_last > 0.1)){
    //                     track(pt);
    //                     geometry_msgs::Pose pose;
    //                     pose.position.x = pt.point.x;
    //                     pose.position.y = pt.point.y;
    //                     cloud.poses.push_back(pose);
    //                 }
    //             }
                
    //             //ROS_INFO("Distance to last obs: %f", dist_to_last);

    //         }else{
    //             ROS_ERROR("Failed to call service: get_human_leg_context.");
    //         }

    //         count_perm++;
    //     }

    //     pub_centers.publish(cloud);
    // }
}

/*
 * Returns true for each person(observation) that must not be considered
 */
bool ParticleFilter::blockObservation(const geometry_msgs::PointStamped &obs) const{
    Vec2f person = Vec2f(obs.point.x, obs.point.y);
    for (int j = 0; j < particles.size(); j++) {
        // if person is inside confidence interval of at least a particle then it is possible it is the target
        bool inside = particles[j]->isInsideConfidenceInterval(person);
        if (inside) {
            return true;
        }
    }
    return false;
}

float ParticleFilter::computeAssociation(const geometry_msgs::Pose &person) const{
    return computeAssociation(person.position);
}

float ParticleFilter::computeAssociation(const geometry_msgs::Point &person) const{
    float association  = 0.0f;
    Vec2f pose;
    pose << person.x, person.y;
    #pragma omp parallel for reduction(+:association)
    for (int i = 0; i < particles.size(); i++) {
        association += particles[i]->confidenceLevel(pose);
    }
    return association;
}

Vec2f ParticleFilter::getCenter(){
    float x = 0;
    float y = 0;

    #pragma omp parallel for reduction(+:x,y)
    for (int i = 0; i < particles.size(); i++) {
        Vec5f state = particles[i]->getState();
        x += state(0);
        y += state(1);
    }

    return Vec2f(x / (float)particles.size(), y / (float)particles.size());
}
