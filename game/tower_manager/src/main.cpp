#include <ros/ros.h>
#include <tower_manager/TowerButtonPressInfo.h>
#include <arduino_publisher/TowerState.h>
#include <tf/transform_broadcaster.h>
#include <tower_manager/TowerInfo.h>
#include <heartbeat/HeartbeatClient.h>
#include <signal.h>
#include <stdlib.h>
#include <time.h>
#include <vector>
#include <string>
#include <map>

class TowerManager{
private:
    bool isExit;

public:

    int num_tower;
    int LED_per_tower;
    int total_LED;
    int global_presses;

    std::vector < std::vector<float> > tower_positions;
    std::map <int, bool> button_state;
    std::map <int, int>  button_counter;
    
    ros::Duration press_time_differ;
    ros::Time last_press_time;
    ros::NodeHandle nh;
    ros::Subscriber sub;

    ros::Publisher pub_tower_info;
    ros::Publisher pub_press_delta;

    TowerManager(): num_tower(0), LED_per_tower(0),
    total_LED(0), global_presses(0), isExit(false)
    {

        pub_tower_info = nh.advertise<tower_manager::TowerInfo>("arduino/tower_info", 1, this);
        pub_press_delta = nh.advertise<tower_manager::TowerButtonPressInfo>("player/tower_button_info", 1, this);
        sub = nh.subscribe("arduino/tower_state", 1, &TowerManager::messageReceived, this);

        nh.getParam("/LED_per_tower", LED_per_tower);
        nh.getParam("/number_of_game_towers", num_tower);

        total_LED = num_tower * total_LED;

        for (int i=0; i < num_tower; i++){
            std::string str = "/tower_" + std::to_string(i+1);
            
            std::vector<float> tower_pos;
            if (!nh.getParam(str, tower_pos)){
                ROS_FATAL_STREAM("Missing parameter: " << "/tower_" << std::to_string(i+1));
                exit(-1);
            }

            tower_positions.push_back(tower_pos);
        }

        initButtonMap();
    }

    // Defines de button_state_map;
    void initButtonMap(){
        for (int i=1; i == (num_tower + 1); i++){
            button_state[i] = false;
            button_counter[i] = 0;
        }
    }

    // A callback function. Executed each time a new tower pose message arrives.
    void messageReceived(const arduino_publisher::TowerState& msg){
        tower_manager::TowerInfo tower_info;
        tower_info.header.stamp = ros::Time::now();
        tower_info.id = msg.id;
        int sum = 0;

        for (auto i: msg.leds){
            sum = sum + i;
        }

        tower_info.completion = (100.0 * sum) / LED_per_tower;
        
        if (button_state[msg.id] != msg.button){
            button_state[msg.id] = msg.button;
            if (msg.button == true){
                button_counter[msg.id] += 1; 
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
                    ROS_DEBUG("First button press registered!");
                }
                /**/
            }
        }

        if (global_presses != 0){
            tower_info.press_rate = button_counter[msg.id]/float(global_presses);
            tower_info.press_counter = button_counter[msg.id];
        }

        pub_tower_info.publish(tower_info);
    }

    void publishTowerTF(){
        for (int i=0; i < num_tower; i++){
            std::string str = "/tower_" + std::to_string(i+1);
            static tf::TransformBroadcaster br;
            tf::Transform transform;
            auto t_pos = tower_positions[i];
            transform.setOrigin( tf::Vector3(t_pos[0],t_pos[1], 0.0) );
            tf::Quaternion q;
            q.setRPY(0, 0, 0);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", str));
        }
    }

};

int main (int argc, char** argv){
    // Initialize the ROS system and become a node .
    ros::init(argc , argv, "tower_manager");

    TowerManager tower_manager;

    // HeartbeatClient Initialize.
    HeartbeatClient hb(tower_manager.nh, 0.2);
	hb.start();
    heartbeat::State::_value_type state = heartbeat::State::INIT;
    hb.setState(state);

    // Loop at 10Hz until the node is shutdown.
    ros::Rate rate(10);
    
    // set heartbeat node state to started
    state = heartbeat::State::STARTED;
    bool success = hb.setState(state);

     while(ros::ok()){
        // Issue heartbeat.
        hb.alive();

        tower_manager.publishTowerTF();

        // Wait until it's time for another iteration.
        rate.sleep();
        ros::spinOnce();
    }
    success = hb.setState(heartbeat::State::STOPPED);
    // Issue heartbeat.
    hb.alive();
    hb.stop();
}
