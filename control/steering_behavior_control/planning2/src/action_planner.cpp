#include "planning2/action_planner.h"
#include "planning2/actions/capture_tower.h"
#include "planning2/actions/escape.h"
#include "planning2/actions/wait.h"
#include "planning2/Action.h"

ActionPlanning::ActionPlanner::ActionPlanner(){
    initMap();

    currentActionId_ = 0;
    currentAction_ = nullptr;

    currentIntentionId_ = -1;

    intentionSub_ = nh_.subscribe("intention", 1, &ActionPlanning::ActionPlanner::intentionCallback, this);
    laserSub_ = nh_.subscribe("/scan", 1, &ActionPlanning::ActionPlanner::laserCallback, this);
    playerPosSub_ = nh_.subscribe("/player_filtered", 1, &ActionPlanning::ActionPlanner::playerPosCallback, this);
    gameStateSub_ = nh_.subscribe("/player_activity", 1, &ActionPlanning::ActionPlanner::gameStateCallback, this);
    actionPub_ = nh_.advertise<planning2::Action>("action", 1);
}

void ActionPlanning::ActionPlanner::initMap(){
    std::vector<ActionPlanning::Action*> playGameActions;
    ActionPlanning::Action* captureTower = new ActionPlanning::CaptureTower();
    playGameActions.push_back(captureTower);

    std::vector<ActionPlanning::Action*> escapeActions;
    ActionPlanning::Action* escape = new ActionPlanning::Escape();
    escapeActions.push_back(escape);

    std::vector<ActionPlanning::Action*> waitActions;
    ActionPlanning::Action* wait = new ActionPlanning::Wait();
    waitActions.push_back(wait);
    

    actions_.insert({0, playGameActions});
    actions_.insert({1, escapeActions});
    actions_.insert({2, waitActions});

}

void ActionPlanning::ActionPlanner::intentionCallback(planning2::Intention iMsg){
    currentIntentionId_ = iMsg.id;
    currentActionId_ += 1;
    std::vector<ActionPlanning::Action*> intentionActions = actions_.at(iMsg.intention_code);

    // currentAction_ = chooseBestAction(intentionActions);
    currentAction_ = intentionActions[0];
    currentAction_->start(currentActionId_, currentIntentionId_);
    currentAction_->generateTargets(currentGameState_, currentPlayerPos_, currentScan_);
    actionPub_.publish(currentAction_->generateActionMessage());
}

void ActionPlanning::ActionPlanner::laserCallback(sensor_msgs::LaserScan scan){
    currentScan_ = scan;
}

void ActionPlanning::ActionPlanner::playerPosCallback(geometry_msgs::PointStamped playerPos){
    currentPlayerPos_ = playerPos;
}

void ActionPlanning::ActionPlanner::gameStateCallback(activity_monitor::Activity gameState){
    currentGameState_ = gameState;
}

void ActionPlanning::ActionPlanner::loop(){
    if(currentAction_ != nullptr){
        std::string actionName = currentAction_->getName();
        ROS_INFO_STREAM(actionName);
        
    }
    else{
        ROS_WARN("Waiting for first intention..");
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "action_planner_node");
    ros::NodeHandle nh;
    ros::Rate rate(30);

    ActionPlanning::ActionPlanner aPlanner;

    while(ros::ok()){
        aPlanner.loop();
        ros::spinOnce();
        rate.sleep();
    }
}