/* Copyright (C) 2015-2018 Ewerton Lopes - All Rights Reserved
*
*  @file    behavior_control.h
*  @author  Ewerton Lopes (ewerton.lopes@polimi.it)
*  @date    04/25/2018  
*  @version 2.0 
*  
*  @brief Implements the Behavior Manager.
*
*  @section DESCRIPTION
*  
*  This is a program that implements the Agent's Behavior Manager.
*  Specifically, this piece of software uses Goal-Oriented Behavior
*  technique to make the decision of which tower to attack next.
*  Upon decision, it publishes the new agent's goal.
*
*  @section LICENSE
*
*  Permission is hereby granted, free of charge, to any person 
*  obtaining a copy of this software and associated documentation
*  files (the "Software"), to deal in the Software without
*  restriction, including without limitation the rights to use, 
*  copy, modify, merge, publish, distribute, sublicense, and/or 
*  sell copies of the Software, and to permit persons to whom the
*  Software is furnished to do so, subject to the following
*  conditions:
*  
*  The above copyright notice and this permission notice shall be
*  included in all copies or substantial portions of the Software.
*
*  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
*  OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
*  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
*  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
*  DEALINGS IN THE SOFTWARE.
*/

#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include <stdlib.h>
#include <string>
#include <time.h>
#include <vector>
#include <cmath>
#include <map>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <arduino_publisher/TowerState.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_listener.h>
#include <behavior_control/Goal.h>
#include <behavior_control/InvBlockInfo.h>
#include <behavior_control/BehaviorParams.h>
#include <nav_msgs/Odometry.h>

#include "gob/gob.h"

namespace Behavior{

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Planner{

private:
    ros::NodeHandle nh_;
    ros::Subscriber tower_state_sub_;
    ros::Publisher goal_pub_;
    ros::Publisher inv_block_pub_;

    // A service for allowing changing the general behavior upon request of other nodes.
    ros::ServiceServer behavior_server_;
    // A service for the calc of inv_block_server_.
    ros::ServiceServer inv_block_server_;

    Behavior::MoveBaseClient* mb_action_client_;
    move_base_msgs::MoveBaseGoal current_mb_goal_;

    tf::TransformListener* tf_listener_;
    
    std::vector<float> blocking_factor_;
    std::vector<float> tower_robot_distances_;
    std::vector<float> tower_player_distances_;
    std::vector<int> leds_on_;

    ros::Time start_decision_timer_;

    std::vector<tf::StampedTransform> towers_robot_transforms_, towers_player_transforms_;
    tf::StampedTransform player_transform_;

    std::vector<gob::Action*> actions_;
    std::vector<gob::Goal*> goals_;

    gob::Action* decision;

    int previous_decision_;
    int target_tower_ID_;
    int num_towers_;
    int num_charg_leds_;
    int bf_exponent_;

    float num_blocks_;
    float block_timeout_;

    bool on_simulation_;
    bool is_game_over_;
    
    double min_dist_to_tower_;
    double max_speed_;
    double min_speed_;

    std::string robot_base_;
    std::string player_base_;

    float euclideanDistance(tf::StampedTransform t);
    float calculateBlockingFactor(tf::StampedTransform player_transform,
                                  tf::StampedTransform tower_robot_transform);
    bool getTransform(std::string target, std::string source, ros::Time &time, tf::StampedTransform &result);
    void getActionIndex(std::string action_name);

public:

    Planner();

    ~Planner(void);
    
    void towerStateCallback(const arduino_publisher::TowerState::ConstPtr& msg);

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    bool changeGameBehavior(behavior_control::BehaviorParams::Request &req,
                            behavior_control::BehaviorParams::Response &resp);
    
    bool calcInvBlockFreq(behavior_control::InvBlockInfo::Request &req,
                            behavior_control::InvBlockInfo::Response &resp);

    void updateDecisionVariables();

    void publishDecision();

    void monitorCollision();

    void updateLoop();

    void makeDecision();

    bool isGameOver();

    bool checkBlockTimeout();

    bool isCancelGoal(int newID);
};

}   // end of namespace

#endif  // BEHAVIOR_H
