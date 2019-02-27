#include <ros/ros.h>
#include <tower_manager/TowerButtonPressInfo.h>
#include <arduino_publisher/TowerState.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
#include <tower_manager/TowerInfo.h>
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

    std::vector <float> leds_turned_on;
    std::vector < std::vector<float> > tower_positions;
    std::map <int, bool> button_state;
    std::map <int, int>  button_counter;
    
    ros::Duration press_time_differ;
    ros::Time last_press_time;
    ros::NodeHandle nh;
    ros::Subscriber sub;

    ros::Publisher pub_tower_info;
    ros::Publisher pub_press_delta;
    ros::Publisher pub_attack_acc_info;

    TowerManager(): num_tower(0), LED_per_tower(0),
    total_LED(0), global_presses(0), isExit(false), leds_turned_on(4)
    {

        //pub_tower_info = nh.advertise<tower_manager::TowerInfo>("arduino/tower_info", 1, this);
        pub_attack_acc_info = nh.advertise<std_msgs::Float64>("player/attack_accuracy", 1, this);
        //pub_press_delta = nh.advertise<tower_manager::TowerButtonPressInfo>("player/tower_button_info", 1, this);
        sub = nh.subscribe("arduino/tower_state", 1, &TowerManager::messageReceived, this);

        nh.getParam("/num_charge_leds_per_tower", LED_per_tower);
        nh.getParam("/num_towers", num_tower);

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


    void calcAttackAccuracy(){
        int sum_bt_presses= 0;
        for (int i=0; i < num_tower; i++){
            sum_bt_presses += button_counter[i];
        }

        int leds_sum = 0;
        for (int i=0; i < leds_turned_on.size(); i++){
            leds_sum = leds_sum + leds_turned_on[i];
        }

        std_msgs::Float64 attack_info;
        
        attack_info.data = leds_sum / float(sum_bt_presses);
        pub_attack_acc_info.publish(attack_info);
    }    

    // A callback function. Executed each time a new tower pose message arrives.
    void messageReceived(const arduino_publisher::TowerState& msg){
        // tower_manager::TowerInfo tower_info;
        // tower_info.header.stamp = ros::Time::now();
        // tower_info.id = msg.id;
        int sum = 0;

        for (auto i: msg.leds){
            sum = sum + i;
        }

        leds_turned_on[msg.id-1] = sum;
        button_counter[msg.id-1] = msg.press_counter;

        // tower_info.completion = sum / LED_per_tower;
        // if (button_state[msg.id] != msg.button){
        //     button_state[msg.id] = msg.button;
        //     if (msg.button == true){
        //         button_counter[msg.id] += 1; 
        //         global_presses += 1;
        //         /* PUBLISH TOWER BUTTON PRESS TIME DIFFERENCE.*/
        //         if (global_presses > 1){
        //             press_time_differ = (msg.header.stamp - last_press_time);
        //             last_press_time = msg.header.stamp;
        //             tower_manager::TowerButtonPressInfo press_msg;
        //             press_msg.header.stamp = ros::Time::now();
        //             press_msg.delta = press_time_differ.toSec();
        //             for (int i=1; i <num_tower+1;i++){
        //                 press_msg.press_distribution.push_back(button_counter[i]/float(global_presses));
        //             }
        //             pub_press_delta.publish(press_msg);
        //         }else if (global_presses == 1){
        //             last_press_time =  msg.header.stamp;
        //             ROS_DEBUG("First button press registered!");
        //         }
        //         /**/
        //     }
        // }

        // if (global_presses != 0){
        //     tower_info.press_rate = button_counter[msg.id]/float(global_presses);
        //     tower_info.press_counter = button_counter[msg.id];
        // }

        // pub_tower_info.publish(tower_info);
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

        static tf::TransformBroadcaster br;
        float x_center = (tower_positions[0][0] + tower_positions[2][0]) / 2.0;
        float y_center = (tower_positions[0][1] + tower_positions[2][1]) / 2.0;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(x_center, y_center, 0.0) );
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "/playground_center"));

    }

};

int main (int argc, char** argv){
    // Initialize the ROS system and become a node .
    ros::init(argc , argv, "tower_manager");

    TowerManager tower_manager;

    // Loop at 10Hz until the node is shutdown.
    ros::Rate rate(10);

     while(ros::ok()){

        tower_manager.publishTowerTF();
        tower_manager.calcAttackAccuracy();

        // Wait until it's time for another iteration.
        rate.sleep();
        ros::spinOnce();
    }
}
