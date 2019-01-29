#include <ros/ros.h>
#include "planning2/intention_planner.h"
#include "planning2/intentions/play_game.h"
#include "planning2/intentions/wait.h"
#include "planning2/intentions/escape.h"


IntentionPlanning::IntentionPlanner::IntentionPlanner(){
    IntentionPlanning::Intention* playGame = new IntentionPlanning::PlayGame();
    IntentionPlanning::Intention* wait = new IntentionPlanning::Wait();
    IntentionPlanning::Intention* escape = new IntentionPlanning::Escape();

    intentions_.insert({PLAY_GAME_CODE, playGame});
    intentions_.insert({WAIT_CODE, wait});
    intentions_.insert({ESCAPE_CODE, escape});

    currentIntention_ = intentions_.at(PLAY_GAME_CODE);
    currentId_ = 0;
    currentIntention_->start(currentId_);
    
    intentionPub_ = nh_.advertise<planning2::Intention>("intention", 1);
    safetySub_ = nh_.subscribe("/safety", 1, &IntentionPlanning::IntentionPlanner::safetyCallback, this);
}

void IntentionPlanning::IntentionPlanner::safetyCallback(planning::SafetyMsg sMsg){
    isSafe_ = sMsg.safety;
}

void IntentionPlanning::IntentionPlanner::loop(){
    std::string current_intention_name = currentIntention_->getName();
    ROS_INFO_STREAM(current_intention_name);

    intentionPub_.publish(currentIntention_->generateIntentionMessage());

    currentIntention_->evaluateIntention(isSafe_);
    int nextIntentionCode = currentIntention_->getNextIntention();
    if(nextIntentionCode != CONTINUE){
        currentId_ += 1;
        currentIntention_ = intentions_.at(nextIntentionCode);
        currentIntention_->start(currentId_);
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "intention_planner_node");
    ros::NodeHandle nh;
    ros::Rate rate(30);

    IntentionPlanning::IntentionPlanner iPlanner;

    while(ros::ok()){
        iPlanner.loop();
        ros::spinOnce();
        rate.sleep();
    }
}