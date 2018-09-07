#include "particle_filter/ParticleFilter.h"
#include "particle_filter/MultipleParticleFilter.h"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <string>

int main(int argc, char** argv){
    
    ros::init(argc, argv, "particle_filter");
    ParticleFilter pf;
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    while (nh.ok()) {
        pf.run();
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();
    return 0;
}