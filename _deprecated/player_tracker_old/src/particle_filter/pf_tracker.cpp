#include "particle_filter/ParticleFilter.h"
#include "particle_filter/MultipleParticleFilter.h"
#include <ros/ros.h>

int main(int argc, char** argv){
    
    ros::init(argc, argv, "particle_filter");
    // ParticleFilter pf;
    MultipleParticleFilter pf;
    ros::spin();
    return 0;
}