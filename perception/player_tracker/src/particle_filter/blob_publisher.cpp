
#include <ros/ros.h>

#include <particle_filter/BlobPublisher.h>

int main(int argc, char** argv){
    
    ros::init(argc, argv, "blob_publisher");

    ros::NodeHandle nh;
    BlobPublisher blobPub(nh, "/blob_publisher");
    ros::spin();

    return 0;
}
