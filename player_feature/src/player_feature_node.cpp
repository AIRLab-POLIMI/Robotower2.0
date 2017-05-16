#include <ros/ros.h>
#include <tower_manager/TowerButtonPressInfo.h>
#include <signal.h>
#include <heartbeat/HeartbeatClient.h>
#include <stdlib.h>
#include <vector>
#include <map>
#include <time.h>
#include <math.h>       /* log2 */

int num_tower(0);
bool isExit = false;

int num_topics = 0;   // signal_size
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
void messageReceived(const tower_manager::TowerButtonPressInfo& msg){
    updateMeanVar(msg.delta);
    ROS_INFO_STREAM("LAST PRESS_DIFF: "<< msg.delta);
    ROS_INFO_STREAM("Mean: " << mean_);
    ROS_INFO_STREAM("Var: " << var_);
    ROS_INFO_STREAM("Entropy: " << entropy(msg.press_distribution));
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

    // HeartbeatClient Initialize.
    HeartbeatClient hb(nh, 0.2);
	hb.start();
    heartbeat::State::_value_type state = heartbeat::State::INIT;
    hb.setState(state);

    // Loop at 100Hz until the node is shutdown.
    ros::Rate rate(100);

    // Create a subscriber object.
    ros::Subscriber sub = nh.subscribe("player/tower_button_info", 1000, &messageReceived);
    // set heartbeat node state to started
    state = heartbeat::State::STARTED;
    bool success = hb.setState(state);

    while(ros::ok() && !isExit){
        // Issue heartbeat.
        hb.alive();
        ros::spinOnce();
        // Wait until it's time for another iteration.
        rate.sleep() ;
    }
    success = hb.setState(heartbeat::State::STOPPED);
    // Issue heartbeat.
    hb.alive();
    hb.stop();
}