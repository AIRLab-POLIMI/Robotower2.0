#include "particle_filter/player_position.h"
#include <vector>


player_tracking::PlayerPositionEstimator::PlayerPositionEstimator(){

    playerCloudSub_ = nh_.subscribe("/player_cloud", 1, &PlayerPositionEstimator::playerCloudCallback, this);
    playerPosPub_ = nh_.advertise<geometry_msgs::Point32>("player", 1);
}

void player_tracking::PlayerPositionEstimator::playerCloudCallback(sensor_msgs::PointCloud playerCloud){
    geometry_msgs::Point32 position = calculateCentroid(playerCloud);
    playerPosPub_.publish(position);
}

geometry_msgs::Point32 player_tracking::PlayerPositionEstimator::calculateCentroid(sensor_msgs::PointCloud playerCloud){
    //centroid try from here
    std::vector<geometry_msgs::Point32> playerPoints = playerCloud.points;
	geometry_msgs::Point32 centroid;
	int x = 0, y = 0, i;
    // ROS_WARN("PlayerCloud size: %d", playerPoints.size());
    if(playerPoints.size()>0){
        for (i = 0; i < playerPoints.size(); i++)
        {
            x += playerPoints[i].x;
            y += playerPoints[i].y;
        }
        centroid.x = x / playerPoints.size();
        centroid.y = y / playerPoints.size();
    }
    else{
        centroid.x = 0;
        centroid.y = 0;
    }
    // ROS_WARN("RETURN CENTROID");
	return centroid;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "player_position_estimation");
    ros::NodeHandle nh;
    ros::Rate rate(10);
	player_tracking::PlayerPositionEstimator estimator;
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }
}

