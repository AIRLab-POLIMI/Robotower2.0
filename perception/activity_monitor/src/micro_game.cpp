#include "activity_monitor/micro_game.h"

Microgame::Microgame::Microgame(){
    gameDuration_ = 8;
    attackNumber_ = 0;

    if (!nh_.getParam("/charging_time", chargingTime_)){
        ROS_ERROR("MODEL MANAGER: could not read 'charging_time' from rosparam!");
        exit(-1);
    }

    startAttackSub_ = nh_.subscribe("/start_attack", 1, &Microgame::Microgame::startAttackCallback, this);
    endAttackSub_ = nh_.subscribe("/end_attack", 1, &Microgame::Microgame::endAttackCallback, this);
    pressTimeSub_ = nh_.subscribe("/tower/press_time", 1, &Microgame::Microgame::pressTimeCallback, this);
    resetSub_ = nh_.subscribe("game_manager/reset", 1, &Microgame::Microgame::resetCallback, this);

    gameOutcomePub_ = nh_.advertise<std_msgs::Int8>("/microgame_outcome", 1);

    last_attack_target_ = -1;
    reset();
}

void Microgame::Microgame::run(){
    ros::Duration elapsedTime_ = ros::Time::now() - gameStartTime_;

    if(attackNumber_ == ATTACK_WINDOW){
        int winner = getWinner();
        if(winner != INVALID){
            gameOutcomePub_.publish(winner);
        }
        reset();
    }

}

void Microgame::Microgame::reset(){
    playerScore_ = 0;
    robotScore_ = 0;
    attackNumber_ = 0;
    hasProgressed_ = false;
    last_attack_target_ = -1;
    first_attack_ = true;

    gameStartTime_ = ros::Time::now();
}

void Microgame::Microgame::evaluateAttackOutcome(){
        if(hasProgressed_){
            playerScore_ += 1;
        }
        else{
            robotScore_ += 1;
        }
        attackNumber_ += 1;
}

void Microgame::Microgame::startAttackCallback(std_msgs::Int8 message){
    // attackNumber_ += 1;
    if(first_attack_){
        first_attack_ = false;
        return;
    }
    if(message.data != last_attack_target_){
        evaluateAttackOutcome();
        hasProgressed_ = false;
        last_attack_target_ = message.data;
    }
    attacking_ = true;
}

void Microgame::Microgame::endAttackCallback(std_msgs::Int8 message){
    attacking_ = false;
}

void Microgame::Microgame::pressTimeCallback(std_msgs::Float64 pressTimeMsg){
    float ledLightedUp = pressTimeMsg.data / chargingTime_;

    if(ledLightedUp >= 1){
        ROS_WARN("PROGRESSING...");
        hasProgressed_ = true;
    }
}

int Microgame::Microgame::getWinner(){
    ROS_WARN("Player score: %d, Robot score: %d", playerScore_, robotScore_);
    int diff = playerScore_ - robotScore_;
    if(diff > 0){
        return PLAYER_WIN;
    }
    else if(diff < 0){
        return ROBOT_WIN;
    }
    else{
        return TIE;
    }

}

void Microgame::Microgame::resetCallback(std_msgs::Bool resetMsg){
    reset();
}

int main(int argc, char** argv){
    ros::init(argc, argv, "microgame_manager_node");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    Microgame::Microgame microgame;

    while(ros::ok()){
        microgame.run();
        ros::spinOnce();
        rate.sleep();
    }
}