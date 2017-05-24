#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>

std::string path_to_sounds;

/** function declarations **/
bool moveToGoal(double xGoal, double yGoal){

   //define a client for to send goal requests to the move_base server through a SimpleActionClient
   actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

   //wait for the action server to come up
   while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
   }

   move_base_msgs::MoveBaseGoal goal;

   //set up the frame parameters
   goal.target_pose.header.frame_id = "map";
   goal.target_pose.header.stamp = ros::Time::now();

   /* moving towards the goal*/

   goal.target_pose.pose.position.x =  xGoal;
   goal.target_pose.pose.position.y =  yGoal;
   goal.target_pose.pose.position.z =  0.0;
   goal.target_pose.pose.orientation.x = 0.0;
   goal.target_pose.pose.orientation.y = 0.0;
   goal.target_pose.pose.orientation.z = 0.0;
   goal.target_pose.pose.orientation.w = 1.0;

   ROS_INFO("Sending goal location ...");
   ac.sendGoal(goal);

   ac.waitForResult();

   if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("You have reached the destination");
      return true;
   }
   else{
      ROS_INFO("The robot failed to reach the destination");
      return false;
   }

}

char choose(){
    char choice = 'q';
    ROS_INFO_STREAM("|-------------------------------|");
    ROS_INFO_STREAM("|PRESSE A KEY:");
    ROS_INFO_STREAM("|'0': Tower1 ");
    ROS_INFO_STREAM("|'1': Tower2 ");
    ROS_INFO_STREAM("|'2': Tower3 ");
    ROS_INFO_STREAM("|'3': Tower4 ");
    ROS_INFO_STREAM("|'q': Quit ");
    ROS_INFO_STREAM("|-------------------------------|");
    ROS_INFO_STREAM("|WHERE TO GO?");
    std::cin >> choice;
    return choice;
}

/** declare the coordinates of interest **/
double xTower1 = -13;
double yTower1 = 3;
double xTower2 = -16;
double yTower2 = 3;
double xTower3 = -16;
double yTower3 = 7;
double xTower4 = -11;
double yTower4 = 7;

bool goalReached = false;

int main(int argc, char** argv){
   ros::init(argc, argv, "game_map_navigation_node");
   ros::NodeHandle n;
   ros::spinOnce();

   char choice = 'q';
   
   do{
      choice =choose();
      if (choice == '0'){
         goalReached = moveToGoal(xTower1, yTower1);
      }else if (choice == '1'){
         goalReached = moveToGoal(xTower2, yTower2);
      }else if (choice == '2'){
         goalReached = moveToGoal(xTower3, yTower3);
      }else if (choice == '3'){
         goalReached = moveToGoal(xTower4, yTower4);
      }
      if (choice!='q'){
         if (goalReached){
            ROS_INFO("Congratulations!");
            ros::spinOnce();
         }else{
            ROS_INFO("Hard Luck!");
         }
      }
   }while(choice !='q');
   return 0;
}
