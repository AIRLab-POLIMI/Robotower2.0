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

#include <stdlib.h>
#include <cmath>
#include <time.h>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <behavior_tree_core/BTAction.h>

#define NUM_TOWERS 4
#define MIN_DIST_TO_TOWER 0.5

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

enum Status {RUNNING, SUCCESS, FAILURE};  // BT return status

tf::TransformListener* tfListener;

class BTAction{
protected:
    ros::NodeHandle nh_;
    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<behavior_tree_core::BTAction> as_;
    std::string action_name_;
    // create messages that are used to published feedback/result
    behavior_tree_core::BTFeedback feedback_;  // action feedback (SUCCESS, FAILURE)
    behavior_tree_core::BTResult result_;  // action feedback  (same as feedback for us)

    // tell the (MoveBase) action client that we want to spin a thread by default
    MoveBaseClient mb_action_client;
    move_base_msgs::MoveBaseGoal current_mb_goal;
    move_base_msgs::MoveBaseGoal previous_mb_goal;

    std::vector<tf::StampedTransform> towers_transforms;    // position with respect to robot.
    tf::StampedTransform player_transform;

    int target_tower_ID;

public:
    explicit BTAction(std::string name) : as_(nh_, name, boost::bind(&BTAction::execute_callback, this, _1), false), mb_action_client("move_base", true),
    action_name_(name), target_tower_ID(1), towers_transforms(NUM_TOWERS){
        // Starts the action server
        as_.start();
  	    
        tfListener = new tf::TransformListener();

        //wait for the action server to come up
        while(!mb_action_client.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up..,");
        }
    }

    ~BTAction(void){}

    void update_decision_variables(){

        /* UPDATE TOWERS AND PLAYER POSTIONS **/
        auto now = ros::Time(0);
    
        // get the position of the towers with respect to the robot
        for(int i = 0; i < NUM_TOWERS; i++){
            try{
                std::string tower_frame = "tower_" + std::to_string(i+1);
                tfListener->waitForTransform(tower_frame, now, "robot_0/base_link", now, "/map", ros::Duration(0.20));
                tfListener->lookupTransform("robot_0/base_link", tower_frame, now, towers_transforms[i]);
            } catch (tf::TransformException ex) {
                ROS_ERROR("%s",ex.what());
            }
        }

        // get the position of the player with respect to the robot
        // try{
        //     tfListener->waitForTransform("/base_link", now, "/player_link", now, "/map", ros::Duration(0.10));
        //     tfListener->lookupTransform("/base_link", "/player_link", now, player_transform);
        // } catch (tf::TransformException ex) {
        //     ROS_ERROR("%s",ex.what());
        // }

        /***/

        /* BLOCKING FACTOR */

    }

    void actuate_decision(){
        /*Publishes the goal position and set the
        navigation parameters (velocity)*/

        // send new move base goal
        current_mb_goal.target_pose.header.frame_id = "tower_" + std::to_string(target_tower_ID);
        current_mb_goal.target_pose.header.stamp = ros::Time::now();
        current_mb_goal.target_pose.pose.orientation.w = 1;

        ROS_INFO("Sending goal");
        mb_action_client.sendGoal(current_mb_goal);

    }

    void monitor_collision(){

    }

    void update_loop(){
        update_decision_variables();
        make_decision();
        actuate_decision();
        monitor_collision();
    }

    void make_decision(){
        int tower_index = target_tower_ID - 1;

        tf::StampedTransform t_transform = towers_transforms[tower_index];
        float x_tower = towers_transforms[tower_index].getOrigin().x(), y_tower = towers_transforms[tower_index].getOrigin().y();

        ROS_INFO_STREAM("Processing goal from TF...");

        if (pow(x_tower, 2) + pow(y_tower, 2) < pow(MIN_DIST_TO_TOWER, 2)){
            ROS_INFO_STREAM("Arrived at goal point...");
            target_tower_ID = (rand() % NUM_TOWERS) + 1;
        }

    }

    void execute_callback(const behavior_tree_core::BTGoalConstPtr &goal){
        // publish info to the console for the user
        ROS_INFO("Setting goal point.");

        //mb_action_client.waitForResult();   <-- this is a blocking function.

        // START EXECUTING THE ACTION

        update_loop();
        
        // check that preempt has not been requested by the BT action client
        if (as_.isPreemptRequested()){
            ROS_INFO("Action Halted");
            
            // set the MoveBase action client state to preempted.
            mb_action_client.cancelGoal();

            // set the BT action state to preempted
            as_.setPreempted();
        }

        if (mb_action_client.getState() == actionlib::SimpleClientGoalState::ACTIVE){
            ROS_INFO("Executing Action");
            set_status(RUNNING);
        }else if(mb_action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO("MoveBase action finished");
            set_status(SUCCESS);    // set BT sucess state 
        }else{
            ROS_INFO("The base failed to move forward 1 meter for some reason");
            set_status(FAILURE);    // set BT FAILURE state
        }

    }

    // returns the status to the client (Behavior Tree)
    void set_status(int status){
        // Set The feedback and result of BT.action
        feedback_.status = status;
        result_.status = feedback_.status;
        // publish the feedback
        as_.publishFeedback(feedback_);
        // setSucceeded means that it has finished the action (it has returned SUCCESS or FAILURE).
        as_.setSucceeded(result_);

        switch (status){  // Print for convenience
            case SUCCESS:
                ROS_INFO("Action %s Succeeded", ros::this_node::getName().c_str() );
                break;
            case FAILURE:
                ROS_INFO("Action %s Failed", ros::this_node::getName().c_str() );
                break;
            default:
                break;
        }
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "behavior_manager");
    ROS_INFO(" Enum: %d", RUNNING);
    ROS_INFO(" Action Ready for Ticks");
    BTAction bt_action(ros::this_node::getName());
    ros::spin();
    return 0;
}
