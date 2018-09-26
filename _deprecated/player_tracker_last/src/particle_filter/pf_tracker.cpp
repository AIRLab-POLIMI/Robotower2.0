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

    ros::Publisher pub_player = nh.advertise<geometry_msgs::Pose>("/player_filtered",1);

    geometry_msgs::PointStamped player_variance;

    while (nh.ok()) {
        pf.run();

        //TODO: use the variances from a yaml file.
        player_variance = pf.getOverallEstimatedVariance();
        if (player_variance.point.x < 0.5 && player_variance.point.y < 0.5)
            pub_player.publish(pf.getPlayerPose());
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();
    return 0;
}