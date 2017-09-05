#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include <vector>
#include <string>

int main(int argc, char** argv){
    ros::init(argc, argv, "tower_tf_broadcaster");
    ros::NodeHandle nh;

    int numTowers;
    nh.getParam("/numberOfTower", numTowers);

    // Loop at 100Hz until the node is shutdown.
    ros::Rate rate(100);

    while(ros::ok()){

        for (int i=0; i < 4; i++){
            std::vector<float> towerPos;
            std::string str = "/tower_" + std::to_string(i+1);
            if (nh.getParam(str, towerPos)){
                static tf::TransformBroadcaster br;
                tf::Transform transform;
                transform.setOrigin( tf::Vector3(towerPos[0],towerPos[1], 0.0) );
                tf::Quaternion q;
                q.setRPY(0, 0, 0);
                transform.setRotation(q);
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", str));
            }else{
                ROS_FATAL_STREAM("Missing parameter: " << "/tower_" << std::to_string(i+1));
            }

        }

        ros::spinOnce();
        // Wait until it's time for another iteration.
        rate.sleep() ;
    }
  return 0;
};