#include <omp.h>
#include <ros/ros.h>
#include <player_tracker/spatial_temporal.h>

#define OMP_THREADS 8

int main(int argc, char **argv) {
    omp_set_num_threads(OMP_THREADS);
    ros::init(argc, argv, "motion_detection_algorithm");

    ros::NodeHandle nh;
    std::string scan_topic;
    nh.param("scan_topic", scan_topic, std::string("scan"));
    spatial_temporal::Extractor ext(nh, scan_topic, 1, "/home/airlab/Scaricati/log_file.txt");

    //spatial_temporal::Extractor ext(nh, scan_topic, 1);

    ros::spin();
    return 0;
}