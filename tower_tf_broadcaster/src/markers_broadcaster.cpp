#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <string>

#define NUM_TOWERS 4

int main(int argc, char** argv){
    ros::init(argc, argv, "tower_tf_broadcaster");
    ros::NodeHandle nh;

    std::vector<ros::Publisher> vis_pubs;

    for (int i=0; i < NUM_TOWERS; i++){
        vis_pubs.push_back(nh.advertise<visualization_msgs::Marker>( "/tower_" + std::to_string(i+1), 0 ));
    }

    // Loop at 10Hz until the node is shutdown.
    ros::Rate rate(10);

    while(ros::ok()){

        // get the position of the towers with respect to the robot
        for(int i = 0; i < NUM_TOWERS; i++){
            auto now = ros::Time(0);
            

                std::string tower_frame = "tower_" + std::to_string(i+1);
    
                // Publish tower markers
                visualization_msgs::Marker marker;
                marker.header.frame_id = tower_frame;
                marker.header.stamp = ros::Time();
                marker.id = i+1;
                marker.type = visualization_msgs::Marker::CYLINDER;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = 0;
                marker.pose.position.y = 0;
                marker.pose.position.z = 0.5;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.scale.z = 1;
                marker.color.a = 1.0; // Don't forget to set the alpha!
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                    
                vis_pubs[i].publish( marker );

        }
        
        ros::spinOnce();
        // Wait until it's time for another iteration.
        rate.sleep() ;
    }
  return 0;
};