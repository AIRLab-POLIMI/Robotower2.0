#include <ros/ros.h>
#include <tower_manager/TowerButtonPressInfo.h>
#include <signal.h>
#include <std_msgs/Float32.h>
#include <stdlib.h>
#include <vector>
#include <map>
#include <time.h>
#include <math.h>       /* log2 */

//Publisher
ros::Publisher pub_mean;
ros::Publisher pub_var;
ros::Publisher pub_entropy;

int num_tower(0);
bool isExit = false;

long int num_topics = 0;   // signal_size
//For Welford's 
float mean_ = 0.0;
float var_ = 0.0;
float s = 0.0;  //var = s / (k + 1)
                
void updateMeanVar(float new_signal_value){
    //Update 
    //Welford's. 
    int oldm = mean_;
    float x = new_signal_value;
    float newm = oldm + (x - oldm) / (num_topics + 1);
    s = s + (x - newm) * (x - oldm);
    mean_ = newm;
    
    var_ = s / (num_topics+1);
    num_topics += 1;
}

// Calculate Shannon entropy.
float entropy(std::vector<double> dist){
    float sum = 0.0;
    for (auto i : dist){
        if (i != 0.0){
            sum += i * log2(i);
        }
    }
    return - (1/log2(num_tower)) * sum;
}

// A callback function. Executed each time a new tower pose message arrives.
// Publishes entropy, mean and var of tower info.
void messageReceived(const tower_manager::TowerButtonPressInfo& msg){
    updateMeanVar(msg.delta);
    std_msgs::Float32 new_msg;
    new_msg.data = mean_;
    pub_mean.publish(new_msg);
    new_msg.data = var_;
    pub_var.publish(new_msg);
    new_msg.data = entropy(msg.press_distribution);
    pub_entropy.publish(new_msg);
}

// Replacement SIGINT handler
void onShutdown(int sig){
    ROS_INFO_STREAM("Exiting...");
    isExit = true;
}

int main (int argc, char** argv){
    // Initialize the ROS system and become a node .
    ros::init(argc , argv, "player_feature");
    ros::NodeHandle nh;
    nh.getParam("/num_tower", num_tower);

    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, onShutdown);

    // Loop at 100Hz until the node is shutdown.
    ros::Rate rate(10);

    // Create a subscriber object.
    ros::Subscriber sub = nh.subscribe("player/tower_button_info", 10, &messageReceived);
    
    // Define publisher object.
    pub_mean    = nh.advertise<std_msgs::Float32>("/tower_button_press_mean", 10);
    pub_var     = nh.advertise<std_msgs::Float32>("/tower_button_press_var", 10);
    pub_entropy = nh.advertise<std_msgs::Float32>("/tower_button_press_entropy", 10);

    while(ros::ok() && !isExit){
        // Wait until it's time for another iteration.
        rate.sleep() ;
        ros::spinOnce();
    }
}
