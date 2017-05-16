#include <ros/ros.h>
#include <arduino_publisher/TowerState.h>
#include <tower_manager/TowerInfo.h>
#include <tower_manager/TowerButtonPressInfo.h>
#include <signal.h>
#include <heartbeat/HeartbeatClient.h>
#include <stdlib.h>
#include <vector>
#include <map>
#include <time.h>

bool isExit = false;

int num_tower;
int LED_per_tower;
int total_LED;
int global_presses(0);      // Global number of button presses

std::map <int, bool> button_state;
std::map <int, int> button_counter;
ros::Time last_press_time;
ros::Duration press_time_differ;

ros::Publisher pub_tower_info;
ros::Publisher pub_press_delta;

// Defines de button_state_map;
void defineButtonMap(){
    for (int i=1; i == (num_tower + 1); i++){
        button_state[i] = false;
        button_counter[i] = 0;
    }
}

// Replacement SIGINT handler
void onShutdown(int sig){
    ROS_INFO_STREAM("Exiting...");
    isExit = true;
}

// A callback function. Executed each time a new tower pose message arrives.
void messageReceived(const arduino_publisher::TowerState& msg){
    tower_manager::TowerInfo tower_info;
    tower_info.header.stamp = ros::Time::now();
    tower_info.id = msg.pipe_id;
    int sum = 0;

    for (auto i: msg.leds){
        sum = sum + i;
    }
    tower_info.completion = (100.0 * sum) / LED_per_tower;
    
    if (button_state[msg.pipe_id] != msg.is_button_pressed){
        button_state[msg.pipe_id] = msg.is_button_pressed;
        if (msg.is_button_pressed == true){
            button_counter[msg.pipe_id] += 1; 
            global_presses += 1;
            /* PUBLISH TOWER BUTTON PRESS TIME DIFFERENCE.*/
            if (global_presses > 1){
                press_time_differ = (msg.header.stamp - last_press_time);
                last_press_time = msg.header.stamp;
                tower_manager::TowerButtonPressInfo press_msg;
                press_msg.header.stamp = ros::Time::now();
                press_msg.delta = press_time_differ.toSec();
                for (int i=1; i <num_tower+1;i++){
                    press_msg.press_distribution.push_back(button_counter[i]/float(global_presses));
                }
                pub_press_delta.publish(press_msg);
            }else if (global_presses == 1){
                last_press_time =  msg.header.stamp;
                ROS_INFO_STREAM("First button press registered!");
            }
            /**/
        }
    }

    if (global_presses != 0){
        tower_info.press_rate = button_counter[msg.pipe_id]/float(global_presses);
        tower_info.press_counter = button_counter[msg.pipe_id];
    }

    pub_tower_info.publish(tower_info);
}

int main (int argc, char** argv){
    // Initialize the ROS system and become a node .
    ros::init(argc , argv, "tower_manager");
    ros::NodeHandle nh;

    nh.getParam("/num_tower", num_tower);
    nh.getParam("/LED_per_tower", LED_per_tower);

    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, onShutdown);

    // HeartbeatClient Initialize.
    HeartbeatClient hb(nh, 0.2);
	hb.start();
    heartbeat::State::_value_type state = heartbeat::State::INIT;
    hb.setState(state);

    ROS_INFO_STREAM("Num of towers: " << num_tower);
    ROS_INFO_STREAM("LEDS per towers: " << LED_per_tower);
    total_LED = num_tower * total_LED;

    defineButtonMap();

    // Loop at 100Hz until the node is shutdown.
    ros::Rate rate(100);
    
    // Create a subscriber object.
    ros::Subscriber sub = nh.subscribe("arduino/tower_state", 1000, &messageReceived);
    // Create a publisher object.
    // Create a publisher object.
    pub_tower_info = nh.advertise<tower_manager::TowerInfo>("arduino/tower_info", 1000);
    pub_press_delta = nh.advertise<tower_manager::TowerButtonPressInfo>("player/tower_button_info", 1000);
    // set heartbeat node state to started
    state = heartbeat::State::STARTED;
    bool success = hb.setState(state);

     while(ros::ok() && !isExit){
        // Issue heartbeat.
        hb.alive();
        ros::spinOnce();
        // Wait until it's time for another iteration.
        rate.sleep();
    }
    success = hb.setState(heartbeat::State::STOPPED);
    // Issue heartbeat.
    hb.alive();
    hb.stop();
}