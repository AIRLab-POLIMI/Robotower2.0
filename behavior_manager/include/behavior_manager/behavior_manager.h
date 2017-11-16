/* Copyright (C) 2015-2017 Ewerton Lopes - All Rights Reserved
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
*   to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
*   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
*   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef BEHAVIOR_MANAGER_H
#define BEHAVIOR_MANAGER_H

#include <stdlib.h>
#include <cmath>
#include <time.h>
#include <vector>
#include <string>
#include <map>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <nav_msgs/Odometry.h>
#include <behavior_manager/Goal.h>
#include <arduino_publisher/TowerState.h>

#include "gob/gob.h"

#define NUM_TOWERS 4
#define NUM_LEDS_PER_TOWER 4
#define MIN_DIST_TO_TOWER 0.5
#define BLOCKING_EXPONENT 5

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

tf::TransformListener* tfListener;

class BehaviorManager{
public:

    ros::NodeHandle nh_;
    ros::Subscriber sub;
    ros::Subscriber tower_state_sub;
    ros::Publisher goal_pub;


    // tell the (MoveBase) action client that we want to spin a thread by default
    MoveBaseClient mb_action_client;
    move_base_msgs::MoveBaseGoal current_mb_goal;

    std::vector<float> blocking_factor;
    std::vector<float> tower_robot_distances;
    std::vector<float> tower_player_distances;
    std::vector<int> leds_on_per_tower;    // number of leds on per tower.

    std::vector<tf::StampedTransform> towers_robot_transforms, towers_player_transforms;    // position with respect to robot.
    tf::StampedTransform player_transform;

    std::vector<gob::Action*> actions;           // action list
    std::vector<gob::Goal*> goals;               // goal list

    int target_tower_ID;
    bool hasWon;
    float robot_speed;
    double max_vel;
    double min_vel;

    int previous_decision;


    BehaviorManager();

    ~BehaviorManager(void);
    
    void towerStateCallback(const arduino_publisher::TowerState::ConstPtr& msg);

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    void update_decision_variables();

    void actuate_decision();

    void monitor_collision();

    void update_loop();

    void make_decision();

    void evaluate_cancel_goal(int newID);
};

#endif  // BEHAVIOR_MANAGER_h
