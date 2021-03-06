#include "behavior_control/planner.h"

#include <std_msgs/Bool.h>

Behavior::Planner::Planner(){
    
    target_tower_ID_ = 1;
    is_game_over_ = false;
    previous_decision_ = -1;
    recent_decision_ = -1;
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
    utilities_.resize(num_towers_);

    behavior_server_ = nh_.advertiseService("change_behavior", &Planner::changeGameBehavior, this);
    inv_block_server_ = nh_.advertiseService("inv_block_info", &Planner::calcInvBlockFreq, this);
    tower_state_sub_ = nh_.subscribe("game_manager/towers/State", 1, &Planner::towerStateCallback, this);
    player_sub_ = nh_.subscribe("/player_filtered", 1, &Planner::playerCallback, this);
    reset_sub_ = nh_.subscribe("/game_manager/reset", 1, &Planner::resetCallback, this);
    
    //get action list
    std::vector<std::string> action_list;
    std::vector<std::string> goal_list;

    was_last_target_.resize(num_towers_);
    was_recent_target_.resize(num_towers_);
    for(int i=0; i<was_last_target_.size(); i++){
        was_last_target_[i] = false;
        was_recent_target_[i] = false;
    }
    towers_left_ = 4;
    
    if (!nh_.getParam("/planner_node/actions", action_list) || !nh_.getParam("/planner_node/goals", goal_list)){
        ROS_ERROR("BEHAVIOR MANAGER: could not read the parameters 'actions' or 'goals' from rosparam! Check behavior_control .yaml file!");
        exit(-1);
    }


    // create action object list from gob namespace
    for (int i=0; i < action_list.size(); i++){
        ROS_DEBUG("ACTION NAME: %s", action_list[i].c_str());
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

        ROS_DEBUG_STREAM(str << ": " << tower_pos[0] << ", " << tower_pos[1] << ", " << tower_pos[2]);
        tower_map_positions[str] = position;
    }

}

void Behavior::Planner::resetCallback(const std_msgs::Bool reset){
    towers_left_ = num_towers_;

    for(int i=0; i<num_towers_; i++){
        leds_on_[i] = 0;
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

// void Behavior::Planner::towerStateCallback(const arduino_publisher::TowerState::ConstPtr& msg){
//     for (int i; i < num_charg_leds_; i ++){
//         leds_on_[msg->id] += msg->leds[i];
//     }
// }

void Behavior::Planner::towerStateCallback(const game_manager::Towers::ConstPtr& msg){
    towers_left_ = 4;
    leds_on_[0] = countLeds(msg->tw1);
    leds_on_[1] = countLeds(msg->tw2);
    leds_on_[2] = countLeds(msg->tw3);
    leds_on_[3] = countLeds(msg->tw4);
}

int Behavior::Planner::countLeds(game_manager::TowerState state){
    int i;
    int count;

    if(state.status == 4){ // Has been captured
        return 4;
        towers_left_ -= 1;
    }

    for(i=0, count=0; i< state.leds.size(); i++){
        if(state.leds[i]){
            count += 1;
        }
    }

    return count;
}

void Behavior::Planner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    ROS_DEBUG("Seq: [%d]", msg->header.seq);
    ROS_DEBUG("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
    ROS_DEBUG("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    ROS_DEBUG("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
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

float Behavior::Planner::euclideanDistance(float delta_x, float delta_y){
    return sqrt(pow(delta_x, 2) + pow(delta_y, 2));
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

void Behavior::Planner::playerCallback(const geometry_msgs::PointStamped pos){
    player_pos_ = pos;
}

void Behavior::Planner::updateDecisionVariables(){

    ROS_DEBUG("Max vel: [%f]", max_speed_);
    ROS_DEBUG("Min vel: [%f]", min_speed_);

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
    float x_player = player_pos_.point.x, y_player = player_pos_.point.y;
    alpha_player = atan2(y_player, x_player);

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
                tf_listener_->waitForTransform(tower_frame, now, "/map", now, "/map", ros::Duration(0.20));
                tf_listener_->lookupTransform("/map", tower_frame, now, towers_player_transforms_[i]);
                
                geometry_msgs::PointStamped player_wrt_map_;
                tf_listener_->waitForTransform("base_link", now, "/map", now, "/map", ros::Duration(0.20));
                tf_listener_->transformPoint("/map", player_pos_, player_wrt_map_);

                tower_player_distances_[i] = euclideanDistance(towers_player_transforms_[i].getOrigin().x() - player_wrt_map_.point.x, towers_player_transforms_[i].getOrigin().y() - player_wrt_map_.point.y);
                /* BLOCKING FACTOR */
                alpha_tower[i] = atan2(y_tower, x_tower);
                blocking_factor_[i] = 1 - 1/M_PI * atan2(fabs(sin(alpha_player - alpha_tower[i])), cos(alpha_player - alpha_tower[i]));
                ROS_DEBUG("blocking_factor[%i] = %1.3f", i, blocking_factor_[i]);
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
                // utility = -tower_player_distances_[i] * pow(blocking_factor_[i], bf_exponent_) - (leds_on_[i]*1000) * (leds_on_[i] == 4); //1000 is a larger weight associated with the number of leds.
                utility = - (tower_robot_distances_[i] - tower_player_distances_[i]) - /** pow(blocking_factor_[i], bf_exponent_)*/ 
                            (leds_on_[i]*1000) * (leds_on_[i] == 4) -
                            (was_recent_target_[i] == true) * (((double)rand()/(RAND_MAX)) < (2.0/3.0)) * (towers_left_ > 2)*1000 -
                            (was_last_target_[i] == true)*(towers_left_ > 1)*1000; //1000 is a larger weight associated with the number of leds.
            }else{
                // utility = -tower_player_distances_[i] - (leds_on_[i]*1000) * (leds_on_[i] == 4); // 1000 is a larger weight associated with the number of leds.
                utility = - (tower_player_distances_[i]) - 
                            (leds_on_[i]*1000) * (leds_on_[i] == 4) - 
                            (was_recent_target_[i] == true)* (((double)rand()/(RAND_MAX)) < (2.0/3.0)) * (towers_left_ > 2)*1000 -
                            (was_last_target_[i] == true)*(towers_left_ > 1)*1000; // 1000 is a larger weight associated with the number of leds.
            }

            
            utilities_[i] = utility;
            float bad_value = -utility;
            actions_[i]->setGoalChange(goals_[j]->getName(), bad_value); // algorithm minimizes Goal Value, 
            ROS_DEBUG_STREAM("Action: " << actions_[i]->getName() << "\tGoal: " << goals_[j]->getName()<< "\tUtility: " << utility);
            // ROS_DEBUG("Player is %f away from tower %d", tower_player_distances_[i], (i+1));
        }
    }

    // is_game_over_ = isGameOver();
 
}


bool Behavior::Planner::isGameOver(){
    // TODO:  Refactor this, it seems it should not be here.
    /*CHECKS WHETHER ROBOT HAS ARRIVED TO TARGET, THUS WINNING THE GAME*/
    int tower_index = target_tower_ID_;
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
        if(nav_mode_ == MOVE_BASE){
            mb_action_client_->cancelGoal();
        }
        has_goal_ = false;
    }
}

void Behavior::Planner::publishDecision(){
    /*Publishes the goal position and set the
    navigation parameters (velocity)*/

    if (!has_goal_){
        ROS_DEBUG("Sending goal...");
        ROS_DEBUG("Publishing Decision");
        if (nav_mode_ == MOVE_BASE){                        // send new move base goal using action lib (move base)
            ROS_DEBUG("Publishing Goal for MOVE BASE");
            std::string t_name = decision->getName();
            current_mb_goal_.target_pose.header.frame_id = t_name.substr(1,t_name.size());
            current_mb_goal_.target_pose.header.stamp = ros::Time::now();
            current_mb_goal_.target_pose.pose.orientation.w = 1;
            mb_action_client_->sendGoal(current_mb_goal_);
            ROS_DEBUG("Goal sent to %s", MOVE_BASE);

        }else{                                               // sends the goal using custom message
            ROS_DEBUG("Publishing Goal");
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
    //ROS_DEBUG("Action Halted");     
    // set the MoveBase action client state to preempted.
    //mb_action_client_->cancelGoal();
}

void Behavior::Planner::updateLoop(){
    
    updateDecisionVariables();

    if (is_game_over_){
        ROS_DEBUG("Game is over!");
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

bool Behavior::Planner::isCancelGoal(int current_decision){
    if (previous_decision_ != current_decision){
        ROS_WARN("Goal changed!");
        for(int i=0; i<was_last_target_.size(); i++){
            was_last_target_[i] = false;
            was_recent_target_[i] = false;
        }
        
        was_last_target_[current_decision] = true;
        if(previous_decision_ >=0){
            was_recent_target_[previous_decision_] = true;
        }

        // recent_decision_ = previous_decision_;
        previous_decision_ = current_decision;
        CancelCurrentGoal();
        
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

std::vector<int> Behavior::Planner::sortTowerIndexes(){
    std::vector<int> indexes_sorted;
    std::vector<float> utilities_copy;

    for(int i=0; i<utilities_.size(); i++){
        indexes_sorted.push_back(i);
        utilities_copy.push_back(utilities_[i]);
    }

    bool has_changed = true;
    while(has_changed){
        has_changed = false;
        for(int j=0; j<utilities_copy.size()-1; j++){
            if(utilities_copy[j] > utilities_copy[j+1]){
                float tmp = utilities_copy[j+1];
                utilities_copy[j+1] = utilities_copy[j];
                utilities_copy[j] = tmp;

                int tmp_idx = indexes_sorted[j+1];
                indexes_sorted[j+1] = indexes_sorted[j];
                indexes_sorted[j] = tmp_idx;

                has_changed = true;
            }
        }
    }

    return indexes_sorted;
}

bool Behavior::Planner::goalRequestHandler(behavior_control::GoalService::Request &req, behavior_control::GoalService::Response &res){
    
    ROS_ERROR("Receiving a request with parameter_id: %d", req.parameter_id);
    updateDecisionVariables();

    if (is_game_over_){
        ROS_DEBUG("Game is over!");
        CancelCurrentGoal();
    }else{
        std::vector<int> sorted_indexes = sortTowerIndexes();
        ROS_WARN_STREAM("Utilities:");
        for(int i=0; i<utilities_.size(); i++){
            ROS_WARN_STREAM(std::to_string(i+1) + ": " + std::to_string(utilities_[i]));
        }

        if(towers_left_ <= 2){
            target_tower_ID_ = sorted_indexes[sorted_indexes.size() - 1];
        }
        else if(req.parameter_id > 1){
            // Choose best one
            target_tower_ID_ = sorted_indexes[sorted_indexes.size() - 2];
        }
        else{
            // Choose second best one
            target_tower_ID_ = sorted_indexes[sorted_indexes.size() - 1];
        }
        isCancelGoal(target_tower_ID_);
        monitorCollision();
    }
    
    res.tower_id = target_tower_ID_ + 1;
    return true;
}
