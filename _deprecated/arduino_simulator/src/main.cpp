#include <ros/ros.h>
#include <arduino_publisher/TowerState.h>
#include <sim_arduino_publisher/Event.h>
#include <std_msgs/Float64.h>
#include <cstdlib>
#include <signal.h>
#include <ctime>
#include <vector>
#define LED_CHARGING_TIME 2.5
#define NUM_LEDS 4


/* TODO: make behavior tree publish the button presses. Call a function to operate the button pressed
Do not forget to make it publish the not press after going out from a tower. We need to know that the button
has been unpressed.*/

namespace Simulation{

typedef struct{
    int id;               // tower id
    bool button;          // is tower button being pressed?
    int status;          // 1: captured, -1: fallen, 0: iddle?
    bool leds[4];         // tower leds array
    //int press_counter;    // number of presses;  <-- NO NEED TO SIMULATE THIS
}TowerStruct;


class Arduino{
private:

    ros::NodeHandle nh_;
    std::vector<TowerStruct*> towers;
    std::clock_t begin_press_time;
    std::clock_t end_press_time;
    ros::Publisher tower_pub;

    bool started;

public:

    Arduino(){
        tower_pub = nh_.advertise<arduino_publisher::TowerState>("arduino/tower_state", 10);
        initTowers();

    }

    ~Arduino(){
        for (int i=0; i < NUM_LEDS ; i++){
            delete towers[i];
        }
    }

    void initTowers(){
        for (int i=0; i < NUM_LEDS; i++){
            TowerStruct* t = new TowerStruct();
            t->id = i;
            t->button = false;
            t->status = 0;
            t->leds[0] = 0;
            t->leds[1] = 0;
            t->leds[2] = 0;
            t->leds[3] = 0;
            towers.push_back(t);
        }
    }

    void process_elapsed_time(const sim_arduino_publisher::Event::ConstPtr &msg){
        double elapsed_secs = double(end_press_time - begin_press_time) / CLOCKS_PER_SEC;
        int num_of_charged = (int) elapsed_secs / LED_CHARGING_TIME;

        int already_charged = 0;

        for (int i=0; i < NUM_LEDS ; i++){
            if (!towers[msg->id]->leds[i]){
                towers[msg->id]->leds[i] = true;
                num_of_charged--;

                if ((i == NUM_LEDS -1) && towers[msg->id]->leds[i])   // all NUM_LEDS charged!
                    towers[msg->id]->status = 1;

            }
        }
        towers[msg->id]->button = false;
    }

    void callback(const sim_arduino_publisher::Event::ConstPtr &msg){

        if (!msg->pressed && started){       // tower button unpressed AFTER being pressed
            if (towers[msg->id]->status == 0){
                end_press_time = clock();
                
                process_elapsed_time(msg);

                started = false;
            }

        }else if (msg->fallen){              // tower has fallen
            towers[msg->id]->status = -1;

        }else{                              // tower button pressed
            if (!started){
                towers[msg->id]->button = true;
                begin_press_time = clock();
                started = true;
            }
        }
    }

    void publish_tower_data(){
        for (int i=0; i < NUM_LEDS ; i++){
            arduino_publisher::TowerState msg;

            // Fill up msg
            msg.header.stamp        = ros::Time::now(); 
            msg.id                  = towers[i]->id;
            msg.button              = towers[i]->button;
            msg.status              = towers[i]->status;
            msg.leds[0]             = towers[i]->leds[0];
            msg.leds[1]             = towers[i]->leds[1];
            msg.leds[2]             = towers[i]->leds[2];
            msg.leds[3]             = towers[i]->leds[3];
            msg.press_counter       = -1;

            // Publish the message.
            tower_pub.publish(msg);
        }
    }
};

}	// end of namespace

int main (int argc, char** argv){
    // Initialize the ROS system and become a node .
    ros::init(argc , argv, "sim_arduino_publisher");
    ros::NodeHandle nh;

    // Create a publisher object.
    Simulation::Arduino arduino_sim;

    // Loop at 100Hz until the node is shutdown.
    ros::Rate rate(10);
    
    while(ros::ok()){
        arduino_sim.publish_tower_data();
        ros::spinOnce();
        // Wait until it's time for another iteration.
        rate.sleep() ;
    }
    
    return 0;
}
