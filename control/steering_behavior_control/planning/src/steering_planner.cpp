#include <ros/ros.h>
#include <activity_monitor/PlayerModel.h>
#include "planning/steering_planning.h"
#include "action/action_lib.h"
#include "planning/SteeringBehaviorEncoded.h"


SteeringPlanning::SteeringPlanner::SteeringPlanner(){
    ROS_INFO("Creating steering planner...");

    if (!nh_.getParam("/planning/action_topic", action_topic_)){
        ROS_ERROR("STEERING PLANNER: could not read 'action_topic' from rosparam!");
        exit(-1);
    }

    if (!nh_.getParam("/planning/behavior_topic", steering_topic_)){
        ROS_ERROR("STEERING PLANNER: could not read 'steering_topic' from rosparam!");
        exit(-1);
    }

    player_model_ = 0.0;

    current_action_msg.action_name = "Stocazzo"; //Useful to bypass null case
    current_action_msg.priority = -1; // Can't be negative by defintion
    
    action_sub_ = nh_.subscribe(action_topic_, 1, &SteeringPlanning::SteeringPlanner::actionCallback, this);
    player_model_sub_ = nh_.subscribe("/player_hyperparam", 1, &SteeringPlanning::SteeringPlanner::playerModelCallback, this);
    steering_pub_ = nh_.advertise<planning::SteeringBehaviorEncoded>(steering_topic_, 1);
}

void SteeringPlanning::SteeringPlanner::actionCallback(const planning::ActionEncoded& msg){
    if(msg.action_name != current_action_msg.action_name){
        if(msg.priority > current_action_msg.priority){
            ROS_WARN("Receiving an action msg whith higher priority...");
            current_action_msg = msg;
            intention_changed_ = true;
            try{
                current_action_ = action_factory_.generateAction(current_action_msg);
            }
            catch(const char* msg){
                ROS_INFO("Waiting for tower positions..");
                current_action_msg.action_name = "Stocazzo"; //Useful to bypass null case
                current_action_msg.priority = -1;
                intention_changed_ = false;
            }
            
        }
    }
    
}

void SteeringPlanning::SteeringPlanner::updateLoop(){
    planning::SteeringBehaviorEncoded steering_msg;

    if(intention_changed_){
        // Publish only if we changed our intention
        intention_changed_ = false;
        if(current_action_msg.action_name == "capture_tower"){            
            int behavior_index = getBestBehaviorIndex(current_action_);
            steering_msg = current_action_->generateSteeringMsg(behavior_index);
            steering_pub_.publish(steering_msg);
        }
        else if(current_action_msg.action_name == "escape"){
            int behavior_index = getBestBehaviorIndex(current_action_);
            steering_msg = current_action_->generateSteeringMsg(behavior_index);
            steering_pub_.publish(steering_msg);
        }
        else if(current_action_msg.action_name == "deceive"){
            int behavior_index = getBestBehaviorIndex(current_action_);
            steering_msg = current_action_->generateSteeringMsg(behavior_index);
            steering_pub_.publish(steering_msg);
        }
    }
}

int SteeringPlanning::SteeringPlanner::getBestBehaviorIndex(Action::AbstractAction* action){
    std::vector<SteeringBehavior::SteeringBehavior*> behaviors = action->getMeaningfulBehaviors();
    int i;
    int best_index;
    float best_value = -1.0; // Can't be negative by definition

    for(i=0; i<behaviors.size(); i++){
        float value = behaviors[i]->evaluate(player_model_); // TODO SET CALLBACK FOR PLAYER MODEL
        if(value > best_value){
            best_index = i;
            best_value = value;
        }
    }

    return best_index;
}

void SteeringPlanning::SteeringPlanner::playerModelCallback(activity_monitor::PlayerModel model){
    player_model_ = model.cumulative_hyperparam;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "planning_steering_node");
    ros::NodeHandle nh;
    ros::Rate rate(30);

    SteeringPlanning::SteeringPlanner planner;

    while(ros::ok()){
        planner.updateLoop();
        ros::spinOnce();
        rate.sleep();
    }
}
