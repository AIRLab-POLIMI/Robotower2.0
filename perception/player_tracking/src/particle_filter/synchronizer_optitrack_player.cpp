#include "particle_filter/player_position.h"
#include "particle_filter/player_ground_truth.h"


typedef message_filters::sync_policies::ApproximateTime<player_tracker::PlayerPositionEstimator, 
                                                        player_tracker::PlayerPositionRepublisher> NoCloudSyncPolicy;

class SynchronizerOptitrackPlayer {
    message_filters::Subscriber<player_tracker::PlayerPositionEstimator> player_position_sub_;      
    message_filters::Subscriber<player_tracker::PlayerPositionRepublisher> optitrack_sub_;      
    message_filters::Synchronizer<NoCloudSyncPolicy>* no_cloud_sync_;
}

void SynchronizerOptitrackPlayer::init()
{

    ros::NodeHandle nh;
    player_position_sub_ = nh_.subscribe("/player_leg_array", 1, &PlayerPositionEstimator::playerLegCallback, this);
    optitrack_sub_ = nh_.subscribe("/player/ground_pose", 1, &PlayerPositionRepublisher::optitrackCallback, this);
    int q = 5; //queue size
    no_cloud_sync_ = new message_filters::Synchronizer<NoCloudSyncPolicy>(NoCloudSyncPolicy(q),  player_position_sub_, optitrack_sub_);
    no_cloud_sync_->registerCallback(boost::bind(&MyClass::callbackMethod, this, _1, _2));
}

//The callback method
void SynchronizerOptitrackPlayer::callbackMethod (player_tracker::PlayerPositionEstimator estimator, 
                                                        player_tracker::PlayerPositionRepublisher republisher) 
{
     //Your processing code
}