#include "behavior_control/planner.h"
#include <std_msgs/Bool.h>

Behavior::Planner::Planner(){
    
    target_tower_ID_ = 1;
    is_game_over_ = false;
    previous_decision_  = 0;
    min_speed_ = .1;
    max_speed_ = 0.2;
    bf_exponent_ = 1;
    num_blocks_ = 0;
    
    tf_listener_ = new tf::TransformListener();

    // read whether we should run on simulation
    if (!nh_.getParam("/planner_node/nav_mode", nav_mode_)){
        ROS_ERROR("BEHAVIOR MANAGER: could not read 'simulation' from rosparam!");
        exit(-1);
    }

    if (nav_mode_ == MOVE_BASE){
        
        mb_action_client_ = new Behavior::MoveBaseClient("move_base", true);

        // wait for the action server to come up
        while(!mb_action_client_->waitForServer(ros::Duration(5.0))){
            ROS_WARN("Waiting for the move_base action server to come up...");
        }

        if (!nh_.getParam("/planner_node/min_dist_to_tower", min_dist_to_tower_)){
            ROS_ERROR("BEHAVIOR MANAGER: could not read 'min_dist_to_tower' from rosparam!");
            exit(-1);
        }

        std::string goal_topic;
        
        if (!nh_.getParam("/planner_node/goal_topic", goal_topic)){
            ROS_ERROR("BEHAVIOR MANAGER: could not read 'goal_topic' from rosparam!");
            exit(-1);
        }

    }else{
        // set publisher for actual game
        std::string goal_topic;

        if (!nh_.getParam("/planner_node/goal_topic", goal_topic)){
            ROS_ERROR("BEHAVIOR MANAGER: could not read 'goal_topic' from rosparam!");
            exit(-1);
        }
        goal_pub_ = nh_.advertise<behavior_control::Goal>(goal_topic.c_str(), 1);
    }
    
    if (!nh_.getParam("/planner_node/robot_base", robot_base_)){
        ROS_ERROR("BEHAVIOR MANAGER: could not read 'robot_base' from rosparam!");
        exit(-1);
    }

    if (!nh_.getParam("/planner_node/player_base", player_base_)){
        ROS_ERROR("BEHAVIOR MANAGER: could not read 'player_base' from rosparam!");
        exit(-1);
    }

    if (!nh_.getParam("/num_towers", num_towers_)){
        ROS_ERROR("BEHAVIOR MANAGER: could not read 'num_towers' from rosparam!");
        exit(-1);
    }

    if (!nh_.getParam("/num_charge_leds_per_tower", num_charg_leds_)){
        ROS_ERROR("BEHAVIOR MANAGER: could not read 'num_charg_leds' from rosparam!");
        exit(-1);
    }

    if (!nh_.getParam("/block_timeout", block_timeout_)){
        ROS_ERROR("BEHAVIOR MANAGER: could not read 'block_timeout' from rosparam!");
        exit(-1);
    }

    blocking_factor_.resize(num_towers_);
    tower_player_distances_.resize(num_towers_);
    tower_robot_distances_.resize(num_towers_);
    towers_robot_transforms_.resize(num_towers_);
    towers_player_transforms_.resize(num_towers_);

    behavior_server_ = nh_.advertiseService("change_behavior", &Planner::changeGameBehavior, this);
    inv_block_server_ = nh_.advertiseService("inv_block_info", &Planner::calcInvBlockFreq, this);
    tower_state_sub_ = nh_.subscribe("/arduino/tower_state", 1, &Planner::towerStateCallback, this);
    
    //get action list
    std::vector<std::string> action_list;
    std::vector<std::string> goal_list;
    
    if (!nh_.getParam("/planner_node/actions", action_list) || !nh_.getParam("/planner_node/goals", goal_list)){
        ROS_ERROR("BEHAVIOR MANAGER: could not read the parameters 'actions' or 'goals' from rosparam! Check behavior_control .yaml file!");
        exit(-1);
    }


    // create action object list from gob namespace
    for (int i=0; i < action_list.size(); i++){
        ROS_INFO("ACTION NAME: %s", action_list[i].c_str());
        gob::Action* action = new gob::Action(action_list[i].c_str());
        actions_.push_back(action);
    }

    // create goal object list from gob namespace
    for (int i=0; i < goal_list.size(); i++){
        gob::Goal* goal = new gob::Goal(goal_list[i], 1/goal_list.size(), 0);
        goals_.push_back(goal);
    }
    
    // set init state for towers.
    for (int i=0; i < num_towers_; i++){
    	leds_on_.push_back(0);
	}

     for (int i=0; i < num_towers_; i++){
        std::string str = "/tower_" + std::to_string(i+1);
        
        std::vector<float> tower_pos;
        if (!nh_.getParam(str, tower_pos)){
            ROS_FATAL_STREAM("Missing parameter: " << "/tower_" << std::to_string(i+1));
            exit(-1);
        }
        
        geometry_msgs::Point* position = new geometry_msgs::Point();
        (*position).x = tower_pos[0];
        (*position).y = tower_pos[1];
        (*position).z = tower_pos[2]; 

        ROS_INFO_STREAM(str << ": " << tower_pos[0] << ", " << tower_pos[1] << ", " << tower_pos[2]);
        tower_map_positions[str] = position;
    }

}

bool Behavior::Planner::calcInvBlockFreq(behavior_control::InvBlockInfo::Request &req, behavior_control::InvBlockInfo::Response &resp){
    
    //send inver_block frequency data
    resp.inv_bloc_frq = (1/num_blocks_ ? num_blocks_ != 0 : 0.0);

    if (req.compute_and_reset){
        num_blocks_ = 0;
    }

    return true;
}

Behavior::Planner::~Planner(void){
    for (int i=0; i < actions_.size(); i++){
        delete actions_[i];
    }

    for (int i=0; i < goals_.size(); i++){
        delete goals_[i];
    }

    delete tf_listener_;
    delete mb_action_client_;

    for (auto it=tower_map_positions.begin(); it!=tower_map_positions.end(); ++it)
        delete it->second;

}

void Behavior::Planner::towerStateCallback(const arduino_publisher::TowerState::ConstPtr& msg){
    for (int i; i < num_charg_leds_; i ++){
        leds_on_[msg->id] += msg->leds[i];
    }
}

void Behavior::Planner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    ROS_INFO("Seq: [%d]", msg->header.seq);
    ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
    ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
}

bool Behavior::Planner::changeGameBehavior(behavior_control::BehaviorParams::Request &req,
                                           behavior_control::BehaviorParams::Response &resp){

    max_speed_ = req.max_speed;
    min_speed_ = req.min_speed;
    bf_exponent_ = req.bf_exponent;

    resp.max_speed = max_speed_;
    resp.min_speed = min_speed_;
    resp.bf_exponent = bf_exponent_;

    return true;
}

float Behavior::Planner::euclideanDistance(tf::StampedTransform t){
    return sqrt(pow(t.getOrigin().x(), 2) + pow(t.getOrigin().y(), 2) + pow(t.getOrigin().z(), 2));
}

bool Behavior::Planner::getTransform(std::string target, std::string source, ros::Time &time, tf::StampedTransform &result){
    try{
        tf_listener_->waitForTransform(target.c_str(), time, source.c_str(), time, "/map", ros::Duration(0.20));
        tf_listener_->lookupTransform(target.c_str(), source.c_str(), time, result);
        return true;
    } catch (const std::exception& ex){
        ROS_WARN("ERROR when getting transform: %s",ex.what());
        return false;
    }
}

void Behavior::Planner::updateDecisionVariables(){

    ROS_INFO("Max vel: [%f]", max_speed_);
    ROS_INFO("Min vel: [%f]", min_speed_);

    /****** UPDATE TOWERS AND PLAYER POSTIONS ******/
    auto now = ros::Time(0);
    std::vector<float> alpha_tower(4);
    float alpha_player = 0;

    bool all_transforms_available = true;
    bool player_transform_available = true;
    
    for (int j=0; j < goals_.size(); j++){
        goals_[j]->setInsistence(1.0 / num_towers_);   //TODO change this constant.
    }

    /* GET THE POSITION OF THE PLAYER WITH RESPECT TO THE ROBOT*/
    try{
        tf_listener_->waitForTransform(robot_base_.c_str(), now, player_base_.c_str(), now, "/map", ros::Duration(0.10));
        tf_listener_->lookupTransform(robot_base_.c_str(), player_base_.c_str(), now, player_transform_);
       
        float x_player = player_transform_.getOrigin().x(), y_player = player_transform_.getOrigin().y();
        alpha_player = atan2(y_player, x_player);

    } catch (const std::exception& ex){
        ROS_WARN("PLANNER NODE: %s does not exist! Skipping using player position for calculating next tower to attack.\n Details: %s",
                 player_base_.c_str(), ex.what());
        //all_transforms_available = false;
        player_transform_available = false;
    }

    /* GET THE POSITION OF THE TOWERS WITHT RESPECT TO THE ROBOT */
    for(int i = 0; i < num_towers_ && all_transforms_available; i++){
        try{

            // toh doh
            std::string tower_frame = "tower_" + std::to_string(i+1);
            tf_listener_->waitForTransform(tower_frame, now, robot_base_.c_str(), now, "/map", ros::Duration(0.20));
            tf_listener_->lookupTransform(robot_base_.c_str(), tower_frame, now, towers_robot_transforms_[i]);

            // get x and y distance to tower
            float x_tower = towers_robot_transforms_[i].getOrigin().x(), y_tower = towers_robot_transforms_[i].getOrigin().y();
            tower_robot_distances_[i] = euclideanDistance(towers_robot_transforms_[i]);
            
            if (player_transform_available) {
                tf_listener_->waitForTransform(tower_frame, now, player_base_.c_str(), now, "/map", ros::Duration(0.20));
                tf_listener_->lookupTransform(player_base_.c_str(), tower_frame, now, towers_player_transforms_[i]);
                tower_player_distances_[i] = euclideanDistance(towers_player_transforms_[i]);
                /* BLOCKING FACTOR */
                alpha_tower[i] = atan2(y_tower, x_tower);
                blocking_factor_[i] = 1 - 1/M_PI * atan2(fabs(sin(alpha_player - alpha_tower[i])), cos(alpha_player - alpha_tower[i]));
                ROS_INFO("blocking_factor[%i] = %1.3f", i, blocking_factor_[i]);
            }


            /* UPDATE GOB VARIABLES*/
            // get x and y distance to tower
            actions_[i]->setDuration(tower_robot_distances_[i] / max_speed_);

        } catch (const std::exception& ex) {
            ROS_ERROR("Could not process tower positions: %s",ex.what());
           all_transforms_available = false;
        }
    }
    
    // if (!all_transforms_available) return;
    for(int i=0; i < num_towers_; i++ ){
        /* update goal change for the corresponding action*/
        for (int j=0; j < goals_.size(); j++){

            float utility;

            if (player_transform_available){
                utility = -tower_player_distances_[i] * pow(blocking_factor_[i], bf_exponent_) - (leds_on_[i]*1000); //1000 is a larger weight associated with the number of leds.
            }else{
                utility = -tower_player_distances_[i] - (leds_on_[i]*1000); //1000 is a larger weight associated with the number of leds.
            }
            float bad_value = -utility;
            actions_[i]->setGoalChange(goals_[j]->getName(), bad_value); // algorithm minimizes Goal Value, 
            ROS_INFO_STREAM("Action: " << actions_[i]->getName() << "\tGoal: " << goals_[j]->getName()<< "\tUtility: " << utility);
        }
    }

    is_game_over_ = isGameOver();
 
}


bool Behavior::Planner::isGameOver(){
    // TODO:  Refactor this, it seems it should not be here.
    /*CHECKS WHETHER ROBOT HAS ARRIVED TO TARGET, THUS WINNING THE GAME*/
    int tower_index = target_tower_ID_ - 1;
    tf::StampedTransform t_transform = towers_robot_transforms_[tower_index];
    float x_tower = towers_robot_transforms_[tower_index].getOrigin().x(), y_tower = towers_robot_transforms_[tower_index].getOrigin().y();
    return (pow(x_tower, 2) + pow(y_tower, 2) < pow(min_dist_to_tower_, 2));
}

float Behavior::Planner::calculateBlockingFactor(tf::StampedTransform player_transform, tf::StampedTransform tower_robot_transform){
    // get x and y distance to tower
    float x_tower = tower_robot_transform.getOrigin().x();
    float y_tower = tower_robot_transform.getOrigin().y();
    float x_player = player_transform.getOrigin().x();
    float y_player = player_transform.getOrigin().y();
    float alpha_tower = atan2(y_tower, x_tower);
    float alpha_player = atan2(y_player, x_player);
    return 1 - 1/M_PI * atan2(fabs(sin(alpha_player - alpha_tower)), cos(alpha_player - alpha_tower));
}

void Behavior::Planner::CancelCurrentGoal(){
    if (has_goal_){
        mb_action_client_->cancelGoal();
        has_goal_ = false;
    }
}

void Behavior::Planner::publishDecision(){
    /*Publishes the goal position and set the
    navigation parameters (velocity)*/

    if (!has_goal_){
        ROS_INFO("Sending goal...");
        
        if (nav_mode_ == MOVE_BASE){                        // send new move base goal using action lib (move base)
            std::string t_name = decision->getName();
            current_mb_goal_.target_pose.header.frame_id = t_name.substr(1,t_name.size());
            current_mb_goal_.target_pose.header.stamp = ros::Time::now();
            current_mb_goal_.target_pose.pose.orientation.w = 1;
            mb_action_client_->sendGoal(current_mb_goal_);
            ROS_INFO("Goal sent to %s", MOVE_BASE);

        }else{                                               // sends the goal using custom message
            behavior_control::Goal goal;
            goal.header.frame_id = decision->getName();
            goal.header.stamp = ros::Time::now();
            goal.tower_number = target_tower_ID_;
            goal_pub_.publish(goal);
        }
        
        has_goal_ = true;
    }
}

void Behavior::Planner::monitorCollision(){
    //ROS_INFO("Action Halted");     
    // set the MoveBase action client state to preempted.
    //mb_action_client_->cancelGoal();
}

void Behavior::Planner::updateLoop(){
    
    updateDecisionVariables();

    if (is_game_over_){
        ROS_INFO("Game is over!");
        CancelCurrentGoal();
    }else{
        makeDecision();
        publishDecision();
        monitorCollision();
    }
}

bool Behavior::Planner::checkBlockTimeout(){
    if ((ros::Time::now().toSec() - start_decision_timer_.toSec()) > block_timeout_){
        return true;
    }else{
        return false;
    }

}

bool Behavior::Planner::isCancelGoal(int new_goal_ID){
    if (previous_decision_ != new_goal_ID){
        ROS_WARN("Goal changed!");
        previous_decision_ = new_goal_ID;
        if (nav_mode_ == MOVE_BASE){
            CancelCurrentGoal();
        }
        num_blocks_ += checkBlockTimeout();     //if no block timeout add one to the count.
        return true;
    }

    return false;
}

void Behavior::Planner::getActionIndex(std::string action_name){
    for(int i=0; i < actions_.size(); i++){
        if (actions_[i]->getName() == action_name){
            target_tower_ID_ = i+1;
            break;
        }
    }
}

void Behavior::Planner::makeDecision(){  
    decision = gob::chooseAction(actions_, goals_);
    getActionIndex(decision->getName());
    
    if (isCancelGoal(target_tower_ID_)){
        start_decision_timer_ = ros::Time::now();
    }

    ROS_WARN("Decision: %s", decision->getName().c_str());
}
