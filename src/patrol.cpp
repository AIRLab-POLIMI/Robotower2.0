#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  std::vector<std::vector<double> > position{{2.1, 2.2, 0.0},{6.5, 4.43, 0.0}};
  std::vector<std::vector<double> > orientation{{0.0, 0.0, 0.0, 1.0},{0.0, 0.0, -0.984047240305, 0.177907360295}};

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  while(true){
  for(int i=0;i<2;i++){
    goal.target_pose.pose.position.x = position[i][0];
    goal.target_pose.pose.position.y = position[i][1];
    goal.target_pose.pose.position.z = position[i][2];
    goal.target_pose.pose.orientation.x = orientation[i][0];
    goal.target_pose.pose.orientation.y = orientation[i][1];
    goal.target_pose.pose.orientation.z = orientation[i][2];
    goal.target_pose.pose.orientation.w = orientation[i][3];

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Hooray, the base moved 1 meter forward");
    else
      ROS_INFO("The base failed to move forward 1 meter for some reason");
  }}

  return 0;
}
