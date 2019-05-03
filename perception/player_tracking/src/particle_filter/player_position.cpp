#include "particle_filter/player_position.h"
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <vector>


player_tracking::PlayerPositionEstimator::PlayerPositionEstimator(){

    // playerCloudSub_ = nh_.subscribe("/player_cloud", 1, &PlayerPositionEstimator::playerCloudCallback, this);
    playerLegSub_ = nh_.subscribe("/player_leg_array", 1, &PlayerPositionEstimator::playerLegCallback, this);
    playerPosPub_ = nh_.advertise<geometry_msgs::PointStamped>("player", 1);
    markerPub_ = nh_.advertise<visualization_msgs::MarkerArray>("filtered_legs", 1);
}

void player_tracking::PlayerPositionEstimator::playerCloudCallback(sensor_msgs::PointCloud playerCloud){
    geometry_msgs::Point32 position = calculateCentroid(playerCloud);
    geometry_msgs::PointStamped positionStamped;
    positionStamped.header.stamp = ros::Time::now();
    positionStamped.header.frame_id = "/map";
    positionStamped.point.x = position.x;
    positionStamped.point.y = position.y;
    playerPosPub_.publish(positionStamped);
}

void player_tracking::PlayerPositionEstimator::playerLegCallback(player_tracker::LegArray legArray){
    
    int leg1Index;
    int leg2Index;
    std::vector<player_tracker::Leg> filteredLegs;
    if(legArray.legs.size() > 2){
        double minDistance = std::numeric_limits<double>::infinity();
        float maxConfidence = -1;
        for(int j=0; j<legArray.legs.size() -1; j++){
            player_tracker::Leg leg1 = legArray.legs[j];
            for(int k=j+1; k<legArray.legs.size(); k++){
                player_tracker::Leg leg2 = legArray.legs[k];
                double currentDistance = calculateLegDistance(leg1, leg2);
                float coupleConfidence = leg1.confidence + leg2.confidence;
                if(currentDistance < 0.8 && currentDistance < minDistance && coupleConfidence>maxConfidence){
                    minDistance = currentDistance;
                    maxConfidence = coupleConfidence;
                    leg1Index = j;
                    leg2Index = k;
                }
            }
        }
        filteredLegs.push_back(legArray.legs[leg1Index]);
        filteredLegs.push_back(legArray.legs[leg2Index]);
    }
    else{
        filteredLegs = legArray.legs;
    }

    visualizePLayerLegArray(filteredLegs);
    sensor_msgs::PointCloud playerCloud;
    std::vector<geometry_msgs::Point32> playerPoints;

    for(int i=0; i<filteredLegs.size(); i++){
        player_tracker::Leg leg;
        leg = filteredLegs[i];
        for(int j=0; j<leg.points.size(); j++){
            playerPoints.push_back(leg.points[j]);
        }
    }
    playerCloud.points = playerPoints;
    playerCloudCallback(playerCloud);
}

double player_tracking::PlayerPositionEstimator::calculateLegDistance(player_tracker::Leg leg1, player_tracker::Leg leg2){
    double distance;
    distance = sqrt(pow(2, leg1.position.x - leg2.position.x) + pow(2, leg1.position.y - leg2.position.y));
    return distance;
}

geometry_msgs::Point32 player_tracking::PlayerPositionEstimator::calculateCentroid(sensor_msgs::PointCloud playerCloud){
    std::vector<geometry_msgs::Point32> playerPoints = playerCloud.points;
	geometry_msgs::Point32 centroid;
	float x = 0, y = 0, i;
    if(playerPoints.size()>0){
        for (i = 0; i < playerPoints.size(); i++)
        {
            x += playerPoints[i].x;
            y += playerPoints[i].y;
        }
        lastKnownPosition_.x = x / playerPoints.size();
        lastKnownPosition_.y = y / playerPoints.size();
    }

	return lastKnownPosition_;
}


void player_tracking::PlayerPositionEstimator::visualizePLayerLegArray(std::vector<player_tracker::Leg> playerLegs){
    visualization_msgs::MarkerArray mArray;
    for(int i=0; i<playerLegs.size(); i++){
        visualization_msgs::Marker marker;
        geometry_msgs::Point position = playerLegs[i].position;
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();
        marker.id = i;

        marker.type = visualization_msgs::Marker::SPHERE;
        
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0f;

        marker.pose.position.x = position.x;
        marker.pose.position.y = position.y;
        marker.pose.position.z = position.z;

        mArray.markers.push_back(marker);
    }
        markerPub_.publish(mArray);
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
