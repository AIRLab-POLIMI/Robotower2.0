#include "particle_filter/Particle.h"


Particle::Particle(const Particle& that) {
    this->state = that.state;
    this->u = that.u;
    this->H = that.H;
    this->I = that.I;
    this->R = that.R;
    this->P = that.P;
    this->F = that.F;
    this->Q = that.Q;
    this->weight = that.weight;
}

Particle::Particle(Vec5f state): weight(0), state(state){

    u << 0,0,0,0,0;

    H << 1,0,0,0,0,           // measurement function 5 states - 3 observed (x, y, theta)
            0,1,0,0,0,
            0,0,1,0,0;  

    I << 1,0,0,0,0,           // identity matrix
            0,1,0,0,0,
            0,0,1,0,0,
            0,0,0,1,0,
            0,0,0,0,1;


    R << 0.5,0,0,               // measurement uncertainty (3 uncorrelated measures with uncertainty)
            0,0.5,0,
            0,0,0.5;

    P << 1,0,0,0,0,             // initial uncertainty
            0,1,0,0,0,
            0,0,1,0,0,
            0,0,0,1,0,
            0,0,0,0,1;            
                                            // Transition Matrix
    F << 1,   0,   0, 0, 0,             // x
            0,   1,   0, 0, 0,             // y
            0,   0,   1, 0, 0,             // theta
            0.1, 0,   0, 1, 0,             // x_dot
            0,   0.1, 0, 0, 1;             // y_dot

    Q << 0.1,0,0,0,0,           // process noise matrix
            0,0.1,0,0,0,
            0,0,0.01,0,0,
            0,0,0,0.01,0,
            0,0,0,0,0.01;
}

Particle::~Particle()
{ /*    */ };

Vec5f Particle::getState() {
    return this->state;
}

/*
 * Propagate using Kalman Predict equation
 */
void Particle::propagate(){
    state = F * state + u;
    P = F*P*F.transpose() + Q;
}

float Particle::distance(Eigen::Vector3f &measure) {
    Eigen::Vector2f pose = state.topRows(2);
    Eigen::Vector2f obs = measure.topRows(2);
    return (pose - obs).norm();
}

float Particle::distance(Eigen::Vector2f &obs) 
{
    Eigen::Vector2f pose = state.topRows(2);
    return (pose - obs).norm();
}

void Particle::fillPose(geometry_msgs::Pose &pose) {
    pose.position.x = state(0);
    pose.position.y = state(1);
    pose.orientation.z = state(2);
}

void Particle::setWeight(float weight) {
    this->weight = weight;
}

float Particle::getWeight() {
    return weight;
}

void Particle::normalizeWeight(float summation) { 
    this->weight /= summation;
}

/*
 * Implements standard Kalman filter update equation
 */
void Particle::update(Eigen::Vector3f &Z) {
    auto Y =  Z - (H*state);
    auto S = ((H*P) * H.transpose()) + R;
    auto K = (P * H.transpose()) * S.inverse();
    state = state + (K * Y);
    P = (I - (K*H)) * P;
}

void Particle::update(Eigen::Vector2f &z) {
    Eigen::Vector3f Z;
    Z << z.x(), z.y(), 0;
    update(Z);
}

ParticlePtr Particle::perturbate(float noiseX, float noiseY, float noiseTheta) const
{
    Particle* perturbated = new Particle(*this);
    perturbated->state(0) += noiseX;
    perturbated->state(1) += noiseY;
    perturbated->state(2) += noiseTheta;

    return perturbated;
}

// new functions
float Particle::confidenceLevel(Vec2f &person)
{
    // ellipsoid (x/sigma_x)^2 + (y/sigma_y)^2 = s (s=5.991 for 95%)
    Vec2f diff = state.topRows(2) - person;
    diff(0) /= P(0, 0); // sigma_x
    diff(1) /= P(1, 1); // sigma_y
    return diff.transpose() * diff;
}

bool Particle::isInsideConfidenceInterval(Vec2f &person)
{
    return confidenceLevel(person) < 5.991f; // true if inside confidence interval of 95%
}
