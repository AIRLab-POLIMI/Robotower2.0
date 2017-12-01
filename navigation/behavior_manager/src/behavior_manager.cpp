#include "behavior_manager/behavior_manager.h"

BehaviorManager::BehaviorManager() : mb_action_client("move_base", true), target_tower_ID(1),
towers_robot_transforms(NUM_TOWERS), towers_player_transforms(NUM_TOWERS), hasWon(false),
previous_decision(0), robot_speed(0), max_vel(0), min_vel(0){
    
    tfListener = new tf::TransformListener();

    blocking_factor.resize(NUM_TOWERS);
    tower_player_distances.resize(NUM_TOWERS);
    tower_robot_distances.resize(NUM_TOWERS);

    //wait for the action server to come up
    // while(!mb_action_client.waitForServer(ros::Duration(5.0))){
    //     ROS_WARN("Waiting for the move_base action server to come up...");
    // }

   goal_pub = nh_.advertise<behavior_manager::Goal>("game/goal", 1000);
   tower_state_sub = nh_.subscribe("/arduino/tower_state", 10, &BehaviorManager::towerStateCallback, this);


    // get action list
    std::vector<std::string> action_list;
    std::vector<std::string> goal_list;
    nh_.getParam("actions", action_list);
    nh_.getParam("goals", goal_list);

    // create action object list from gob namespace
    for (int i=0; i < action_list.size(); i++){
        ROS_INFO("ACTION NAME: %s", action_list[i].c_str());
        gob::Action* action = new gob::Action(action_list[i].c_str());
        actions.push_back(action);
    }

    // create goal object list from gob namespace
    for (int i=0; i < goal_list.size(); i++){
        gob::Goal* goal = new gob::Goal(goal_list[i], 1/goal_list.size(), 0);
        goals.push_back(goal);
    }
    
    // set init state for towers.
    for (int i=0; i < NUM_TOWERS; i++){
    	leds_on_per_tower.push_back(0);
	}

}

BehaviorManager::~BehaviorManager(void){
    for (int i=0; i < actions.size(); i++){
        delete actions[i];
    }

    for (int i=0; i < goals.size(); i++){
        delete goals[i];
    }
}

void BehaviorManager::towerStateCallback(const arduino_publisher::TowerState::ConstPtr& msg){
    for (int i; i < NUM_LEDS_PER_TOWER; i ++){
        leds_on_per_tower[msg->pipe_id] += (msg->leds[i] ? 1 : 0);
    }
}

void BehaviorManager::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    ROS_INFO("Seq: [%d]", msg->header.seq);
    ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
    ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
}


float euclidean_distance(tf::StampedTransform t){
    return sqrt(pow(t.getOrigin().x(), 2) + pow(t.getOrigin().y(), 2) + pow(t.getOrigin().z(), 2));
}

void BehaviorManager::update_decision_variables(){

    nh_.getParam("/max_vel_x",max_vel);
    nh_.getParam("/min_vel_x",min_vel);
    ROS_INFO("Max vel: [%f]", max_vel);
    ROS_INFO("Min vel: [%f]", min_vel);

    /****** UPDATE TOWERS AND PLAYER POSTIONS ******/
    auto now = ros::Time(0);
    std::vector<float> alpha_tower(4);
    float alpha_player = 0;

    bool all_transforms_available = true;
    
    for (int j=0; j < goals.size(); j++){
        goals[j]->setValue(1.0 / NUM_TOWERS);   //TODO change this constant.
    }

    /* GET THE POSITION OF THE PLAYER WITH RESPECT TO THE ROBOT*/
    try{
        tfListener->waitForTransform("base_link", now, "player_link", now, "/map", ros::Duration(0.10));
        tfListener->lookupTransform("base_link", "player_link", now, player_transform);
       
        float x_player = player_transform.getOrigin().x(), y_player = player_transform.getOrigin().y();
        alpha_player = atan2(y_player, x_player);

    } catch (const std::exception& ex){
        ROS_WARN("%s",ex.what());
        return; // all_transforms_available = false;
    }

    /* GET THE POSITION OF THE TOWERS WITHT RESPECT TO THE ROBOT */
    for(int i = 0; i < NUM_TOWERS && all_transforms_available; i++){
        try{

            // toh doh
            std::string tower_frame = "tower_" + std::to_string(i+1);
            tfListener->waitForTransform(tower_frame, now, "base_link", now, "/map", ros::Duration(0.20));
            tfListener->lookupTransform("base_link", tower_frame, now, towers_robot_transforms[i]);

            // toh doh
            tfListener->waitForTransform(tower_frame, now, "/player_link", now, "/map", ros::Duration(0.20));
            tfListener->lookupTransform("player_link", tower_frame, now, towers_player_transforms[i]);

            // get x and y distance to tower
            float x_tower = towers_robot_transforms[i].getOrigin().x(), y_tower = towers_robot_transforms[i].getOrigin().y();
            
            /* BLOCKING FACTOR */
            alpha_tower[i] = atan2(y_tower, x_tower);

            blocking_factor[i] = 1 - 1/M_PI * atan2(fabs(sin(alpha_player - alpha_tower[i])), cos(alpha_player - alpha_tower[i]));
            tower_robot_distances[i] = euclidean_distance(towers_robot_transforms[i]);
            tower_player_distances[i] = euclidean_distance(towers_player_transforms[i]);

            /* UPDATE GOB VARIABLES*/
            // get x and y distance to tower
            actions[i]->updateDuration(tower_robot_distances[i] / max_vel);

            ROS_INFO("blocking_factor[%i] = %1.3f", i, blocking_factor[i]);
            //ROS_INFO("x_tower = %f,\ty_tower = %f", x_tower, y_tower);
            //ROS_INFO("alpha_tower[%i] = %f", i, alpha_tower[i]);

        } catch (const std::exception& ex) {
            ROS_ERROR("%s",ex.what());
           all_transforms_available = false;
        }
    }
    
    // if (!all_transforms_available) return;

    for(int i=0; i < NUM_TOWERS; i++ ){
        /* update goal change for the corresponding action*/
        for (int j=0; j < goals.size(); j++){
            float utility = -tower_player_distances[i] * pow(blocking_factor[i], BLOCKING_EXPONENT) - (leds_on_per_tower[i]*1000); //1000 is a larger weight associated with the number of leds.
            float bad_value = -utility;
            actions[i]->updateGoalChange(goals[j]->getName(), bad_value); // algorithm minimizes Goal Value, 
            ROS_INFO_STREAM("Action: " << actions[i]->getName() << "\tGoal: " << goals[j]->getName()<< "\tUtility: " << utility);
        }
    }


    /*CHECKS WHETHER ROBOT HAS ARRIVED TO TARGET, THUS WINNING THE GAME*/
    int tower_index = target_tower_ID - 1;
    tf::StampedTransform t_transform = towers_robot_transforms[tower_index];
    float x_tower = towers_robot_transforms[tower_index].getOrigin().x(), y_tower = towers_robot_transforms[tower_index].getOrigin().y();
    hasWon = (pow(x_tower, 2) + pow(y_tower, 2) < pow(MIN_DIST_TO_TOWER, 2));
    /**/
    
}

void BehaviorManager::actuate_decision(){
    /*Publishes the goal position and set the
    navigation parameters (velocity)*/

    // send new move base goal
    // ACTION LIB
    //current_mb_goal.target_pose.header.frame_id = "tower_" + std::to_string(target_tower_ID);
    //current_mb_goal.target_pose.header.stamp = ros::Time::now();
    //current_mb_goal.target_pose.pose.orientation.w = 1;
    // mb_action_client.sendGoal(current_mb_goal);
    
    
    behavior_manager::Goal goal;
    goal.header.frame_id = "tower_" + std::to_string(target_tower_ID);
    goal.header.stamp = ros::Time::now();
    goal.tower_number = target_tower_ID;
    ROS_INFO("Sending goal");
    goal_pub.publish(goal);

}

void BehaviorManager::monitor_collision(){
    //ROS_INFO("Action Halted");     
    // set the MoveBase action client state to preempted.
    //mb_action_client.cancelGoal();
}

void BehaviorManager::update_loop(){
    
    update_decision_variables();

    if (hasWon){
        ROS_INFO("Has won!");
        //mb_action_client.cancelGoal();
    }else{
        make_decision();
        actuate_decision();
        monitor_collision();
    }
}

void BehaviorManager:: evaluate_cancel_goal(int newID){
    if (previous_decision != newID){
        ROS_INFO("Goal changed!");
        previous_decision = newID;
        //mb_action_client.cancelGoal();
    }
}

void BehaviorManager::make_decision(){  
    gob::Action* decided = gob::chooseAction(actions, goals);

    ROS_INFO("Decision: %s", decided->getName().c_str());

    if (decided->getName() == "gTower1"){
        target_tower_ID = 1;
        evaluate_cancel_goal(target_tower_ID);
    }else if (decided->getName() == "gTower2"){
        target_tower_ID = 2;
        evaluate_cancel_goal(target_tower_ID);
    }else if (decided->getName() == "gTower3"){
        target_tower_ID = 3;
        evaluate_cancel_goal(target_tower_ID);
    }else if (decided->getName() == "gTower4"){
        target_tower_ID = 4;
        evaluate_cancel_goal(target_tower_ID);
    }else{
        ROS_INFO("Wait is the best action!");
    }

}
