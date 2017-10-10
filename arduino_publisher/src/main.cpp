#include <ros/ros.h>
#include <arduino_publisher/TowerState.h>
#include <arduino_publisher/ImuState.h>
#include <std_msgs/Float64.h>
#include <stdlib.h>
#include <time.h>
#include <signal.h>
#include <heartbeat/HeartbeatClient.h>
#include "SerialPort.h"

/* Pipe indexes*/
#define TOWER1_INDEX 1
#define TOWER2_INDEX 2
#define TOWER3_INDEX 3
#define TOWER4_INDEX 4
#define ACC_INDEX 5
#define BAT_INDEX 6
/* **** */

/* Data sizes */
#define TOWER_ARRAY_SIZE 14
#define ACC_ARRAY_SIZE 12

bool isExit = false;

template<typename Out>
void split(const std::string &s, char delim, Out result) {
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        *(result++) = item;
    }
}

std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, std::back_inserter(elems));
    return elems;
}

// Replacement SIGINT handler
void onShutdown(int sig){
    ROS_INFO_STREAM("Exiting...");
    isExit = true;
}

int main (int argc, char** argv){
    // Initialize the ROS system and become a node .
    ros::init(argc , argv, "arduino_publisher", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, onShutdown);

    // HeartbeatClient Initialize.
    HeartbeatClient hb(nh, 0.5);
	hb.start();

    heartbeat::State::_value_type state = heartbeat::State::INIT;
    hb.setState(state);

    std::vector<std::string> formatVector;
    nh.getParam("/format", formatVector);

    // Create a publisher object.
    ros::Publisher tower_pub = nh.advertise
                              <arduino_publisher::TowerState>
                              ("arduino/tower_state", 1000);
    ros::Publisher imu_pub = nh.advertise
                             <arduino_publisher::ImuState>
                             ("arduino/imu_state", 1000);
    ros::Publisher bat_pub = nh.advertise<std_msgs::Float64>("battery_state",1000);

    // Loop at 100Hz until the node is shutdown.
    ros::Rate rate(100);

    // Define serial communication object
    arduino_serial::SerialPort serialCom("/dev/arduino");
    
    // delimeter used for tagging the end of arduino string.
    const char* SERIAL_COM_DELIMITER = "\n";
    
    // set heartbeat node state to started
    state = heartbeat::State::STARTED;
    bool success = hb.setState(state);

    while(ros::ok() && !isExit){
        // Issue heartbeat.
        hb.alive();

        std::stringstream buffer;

        // read port
        int bytes = serialCom.read_port_until(buffer, SERIAL_COM_DELIMITER, 5000);
        std::vector<std::string> data = split(buffer.str().c_str(),',');

        if (bytes > 0){
            /* Look at the first string value received and 
            choose the appropriate data type for publish */
            if ((atof(data[0].c_str()) == TOWER1_INDEX) || (atof(data[0].c_str()) == TOWER2_INDEX) ||
                (atof(data[0].c_str()) == TOWER3_INDEX) || (atof(data[0].c_str()) == TOWER4_INDEX) ){           /* towers */
                
                if (data.size() == TOWER_ARRAY_SIZE){
                    try{ 
                        // Create and fill in the message.
                        arduino_publisher::TowerState msg;
                   
                        // Fill up msg
                        msg.header.stamp        = ros::Time::now(); 
                        msg.pipe_id             = atof(data[0].c_str());
                        msg.is_tower_enable     = atof(data[1].c_str());
                        msg.is_button_pressed   = atof(data[2].c_str());
                        msg.is_captured         = atof(data[3].c_str());
                        msg.is_tower_down       = atof(data[4].c_str());
                        msg.ultrasounds[0]      = atof(data[5].c_str());
                        msg.ultrasounds[1]      = atof(data[6].c_str());
                        msg.ultrasounds[2]      = atof(data[7].c_str());
                        msg.leds[0]             = atof(data[8].c_str());
                        msg.leds[1]             = atof(data[9].c_str());
                        msg.leds[2]             = atof(data[10].c_str());
                        msg.leds[3]             = atof(data[11].c_str());
                        msg.press_counter       = atof(data[12].c_str());
                    
                        // Publish the message.
                        tower_pub.publish(msg);
                    } catch (...){
                        ROS_ERROR("An error has occurred when reading accelerometer data!");
                    }
                }else{
                    ROS_WARN("TOWER data doesn't have the required size! Received: %s", buffer.str().c_str());
                }

            } else if ((atof(data[0].c_str()) == BAT_INDEX)){
                // Create and fill in the message.
                try{
                    std_msgs::Float64 bat_msg;
                    // TODO: the line 140 causes a program crash in case no battery is attached to the robot.
                    // This is mostly because in that case, the arduino receiver code sends rubish in the voltage
                    // variable, since the associated analog pin may contain rubish.
                    bat_msg.data            = atof(data[1].c_str());
                    // Publish the message.
                    bat_pub.publish(bat_msg);
                } catch (...){
                    ROS_ERROR("An error has occurred when reading accelerometer data!");
                }

            } else if ((atof(data[0].c_str()) == ACC_INDEX)){ /*accelerometer*/
                // Create and fill in the message.
                arduino_publisher::ImuState msg;
                if (data.size() == ACC_ARRAY_SIZE){   
                    try{  
                            // Fill up msg
                            msg.header.stamp = ros::Time::now();
                            msg.linear_acc.x = atof(data[1].c_str());
                            msg.linear_acc.y = atof(data[2].c_str());
                            msg.linear_acc.z = atof(data[3].c_str());
                            msg.gyro.x = atof(data[4].c_str());
                            msg.gyro.y = atof(data[5].c_str());
                            msg.gyro.z = atof(data[6].c_str());
                            msg.q[0]   = atof(data[7].c_str());
                            msg.q[1]   = atof(data[8].c_str());
                            msg.q[2]   = atof(data[9].c_str());
                            msg.q[3]   = atof(data[10].c_str());
                            // Publish the message.
                            imu_pub.publish(msg);
                    } catch (...){
                        ROS_ERROR("An error has occurred when reading accelerometer data!");
                    }
                }else{
                    ROS_WARN("ACCELEROMETER data doesn't have the required size! Received: %s", buffer.str().c_str());
                }
            }

		}else{
			ROS_ERROR("ARDUINO_PUBLISHER: Could not read_port! Is it connected?");
		}

        ros::spinOnce();
        // Wait until it's time for another iteration.
        rate.sleep() ;
    }
    success = hb.setState(heartbeat::State::STOPPED);
    // Issue heartbeat.
    hb.alive();
    hb.stop();
    return 0;
}
