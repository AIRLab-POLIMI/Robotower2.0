#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include "activity_monitor/player_model_manager.h"
#include "activity_monitor/PlayerModel.h"

#include <algorithm>

#define CLAMPING_CONSTANT 1 // Constant to clamp inverse of distance
#define EXPERTISE_WINDOW_LENGTH 3

ModelingManager::PlayerModelManager::PlayerModelManager(){
    // CONSTRUCTOR
    cumulative_estimate_ = 0.0;
    count_ = 0;
    previous_index_to_monitor_ = -1;
    target_index_to_monitor_ = -1;

    total_risk_taken_ = 0.0;
    count_risk_ = 0;

    if (!nh_.getParam("/charging_time", charging_time_)){
            ROS_ERROR("MODEL MANAGER: could not read 'charging_time' from rosparam!");
            exit(-1);
    }

    double starting_speed;
    if (!nh_.getParam("/player_activity_monitor/base_speed", starting_speed) ){
        ROS_ERROR("VEHICLE: could not read the parameter 'base_speed' from rosparam! Check steerning_behavior .yaml file!");
        exit(-1);
    }

    expertise_window_.resize(EXPERTISE_WINDOW_LENGTH);
    expertise_sum_ = 0.0;
    expertise_index_ = 0;
    full_ = false;

    last_kinematic_update_msg_.current_speed = starting_speed;
    charging_time_ = charging_time_ * 1000;

    player_activity_sub_ = nh_.subscribe("/player_activity", 1, &ModelingManager::PlayerModelManager::playerActivityCallback, this);
    start_interaction_sub_ = nh_.subscribe("/start_interaction", 1, &ModelingManager::PlayerModelManager::startInteractionCallback, this);
    abort_interaction_sub_ = nh_.subscribe("/abort_interaction", 1, &ModelingManager::PlayerModelManager::abortInteractionCallback, this);
    reset_sub_ = nh_.subscribe("/game_manager/reset", 1, &ModelingManager::PlayerModelManager::resetCallback, this);
    press_time_sub_ = nh_.subscribe("/tower/press_time", 1, &ModelingManager::PlayerModelManager::pressTimeCallback, this);
    
    model_pub_ = nh_.advertise<activity_monitor::PlayerModel>("/player_model", 1);


}

void ModelingManager::PlayerModelManager::playerActivityCallback(activity_monitor::Activity message){
    last_activity_msg_ = message;
}

void ModelingManager::PlayerModelManager::startInteractionCallback(std_msgs::Int8 message){
    if(target_index_to_monitor_ == -1){
        target_index_to_monitor_ = message.data;
        is_monitoring_ = false;
        ROS_WARN("FIRST ONE, NOT MONITORING");
    }
    else{
        previous_index_to_monitor_ = target_index_to_monitor_;
        if(target_index_to_monitor_ == message.data){
            // We have not changed the  target, do not monitor
            is_monitoring_ = false;
            ROS_WARN("SAME TARGET, NOT MONITORING");
        }
        else{
            is_monitoring_ = true;
            starting_time_ = ros::Time::now().toSec();
            estimated_time_arrival_ = (last_activity_msg_.robot_tower_distances[message.data] / last_kinematic_update_msg_.current_speed) * 0.6;
            ROS_INFO("MONITORING");
        }
        target_index_to_monitor_ = message.data;
    }    
}

void ModelingManager::PlayerModelManager::abortInteractionCallback(std_msgs::Int8 message){
    is_monitoring_ = false;
}

void ModelingManager::PlayerModelManager::kinematicUpdateCallback(steering_behavior::KinematicUpdate msg){
    last_kinematic_update_msg_ = msg;
}

double ModelingManager::PlayerModelManager::calculateTowerAttraction(int tower){
    double attraction = 1.0 / (last_activity_msg_.player_tower_distances[tower] + CLAMPING_CONSTANT);
    ROS_INFO("Tower %d attraction: %.2f", tower+1, attraction);
    return attraction;
}

double ModelingManager::PlayerModelManager::calculateRobotPerturbation(int tower){
    double distance_perturbation = 1.0 / (last_activity_msg_.robot_tower_distances[tower] + CLAMPING_CONSTANT);
    double velocity_perturbation = last_activity_msg_.velocity_projections[tower];
    ROS_INFO("Robot %d distance perturbation: %.2f and velocity: %.2f", tower+1, distance_perturbation, velocity_perturbation);
    return distance_perturbation + velocity_perturbation;
}

double ModelingManager::PlayerModelManager::estimateHyperparameter(){
    ROS_INFO("Switching from tower %d to tower %d", previous_index_to_monitor_+1, target_index_to_monitor_+1);
    // double delta_tower_static_attraction = calculateTowerAttraction(target_index_to_monitor_) - calculateTowerAttraction(last_activity_msg_.closest_tower_index);
    // double delta_robot_perturbation = calculateRobotPerturbation(last_activity_msg_.closest_tower_index) - calculateRobotPerturbation(target_index_to_monitor_);

    double delta_tower_static_attraction = calculateTowerAttraction(target_index_to_monitor_) - calculateTowerAttraction(previous_index_to_monitor_);
    double delta_robot_perturbation = calculateRobotPerturbation(previous_index_to_monitor_) - calculateRobotPerturbation(target_index_to_monitor_);

    return delta_robot_perturbation / delta_tower_static_attraction;
}

activity_monitor::PlayerModel ModelingManager::PlayerModelManager::updateModel(double new_estimate){
    // TODO AVERAGE ALL ESTIMATES
    ROS_WARN("NEW ESTIMATE: %.2f", new_estimate);
    // if(new_estimate > 6.0 or new_estimate < 0.0){
    //     // Something when wrong in recognition
    //     return;
    // }
    cumulative_estimate_ += new_estimate;
    count_ += 1;
    tower_over_robot_attraction_ = cumulative_estimate_ / count_;

    // std_msgs::Float64 model_message;
    activity_monitor::PlayerModel model_message;
    model_message.cumulative_hyperparam = tower_over_robot_attraction_;
    model_message.current_estimate = new_estimate;
    model_message.count = count_;
    
    return model_message;
}

void ModelingManager::PlayerModelManager::run(){
    if(is_monitoring_){
        if(!last_activity_msg_.is_stationary){
            if(target_index_to_monitor_ == last_activity_msg_.target_tower){
                // When the player understands robot's target
                updateModel(estimateHyperparameter());
                is_monitoring_ = false;
            }else{
                ROS_WARN("WRONG TARGET");
            }
        }
    }
}

void ModelingManager::PlayerModelManager::resetCallback(std_msgs::Bool reset_msg){
    ROS_INFO("RESETTING");
    average_led_per_press_ = 0.0;
    total_led_per_press_ = 0.0;
    count_press_ = 0;
    
    cumulative_estimate_ = 0.0;
    count_ = 0;
    
    total_risk_taken_ = 0.0;
    count_risk_ = 0;
    
    target_index_to_monitor_ = -1;
    previous_index_to_monitor_ = -1;

    
    is_monitoring_ = false;
}

double ModelingManager::PlayerModelManager::estimateRisk(){
    return (last_activity_msg_.velocity_projections[target_index_to_monitor_] * 5)/ (last_activity_msg_.robot_tower_distances[target_index_to_monitor_] + 0.2);
}

void ModelingManager::PlayerModelManager::pressTimeCallback(std_msgs::Float64 press_time_msg){
    if(is_monitoring_){
        double led = press_time_msg.data / charging_time_;
        total_led_per_press_ = total_led_per_press_ + led;
        count_press_ = count_press_ + 1;
        average_led_per_press_ = total_led_per_press_ / count_press_;

        double risk = estimateRisk();
        total_risk_taken_ = total_risk_taken_ + risk;
        count_risk_ = count_risk_ + 1;
        average_risk_taken_ = total_risk_taken_ / count_risk_;

        is_monitoring_ = false;

        activity_monitor::PlayerModel model_message = updateModel(estimateHyperparameter());
        model_message.average_led_per_press = average_led_per_press_;
        model_message.last_led_per_press = led;

        model_message.average_risk_taken = average_risk_taken_;
        model_message.last_risk_taken = risk;

        float expertise = (ros::Time::now().toSec() - starting_time_) / estimated_time_arrival_;
        if(expertise >= 1.0){
            expertise = 1.0;
        }
        expertise_sum_ = expertise_sum_ - expertise_window_[expertise_index_] + expertise;
        expertise_window_[expertise_index_] = expertise;
        expertise_index_ += 1;
        if(expertise_index_ == EXPERTISE_WINDOW_LENGTH){
            full_ = true;
        }
        float current_expertise_estimate;
        if(full_){
            current_expertise_estimate = expertise_sum_ / EXPERTISE_WINDOW_LENGTH;
        }
        else{
            current_expertise_estimate = expertise_sum_ / expertise_index_;
        }
        expertise_index_ = expertise_index_ % EXPERTISE_WINDOW_LENGTH;


        model_message.expertise = current_expertise_estimate; 
        model_message.last_expertise = expertise;
        // model_message.count = count_;
        model_pub_.publish(model_message);
    }
}


int main(int argc, char** argv){
    ros::init(argc, argv, "player_model_manager_node");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    ModelingManager::PlayerModelManager model_manager;

    while(ros::ok()){
        // model_manager.run();
        ros::spinOnce();
        rate.sleep();
    }
}