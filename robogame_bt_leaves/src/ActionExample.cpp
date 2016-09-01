#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>// needed for actionlib
#include <robogame_bt_leaves/BTAction.h>//Definition of action. see /scr/action



enum Status {RUNNING,SUCCESS, FAILURE};//BT return status


class BTAction
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<robogame_bt_leaves::BTAction> as_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  robogame_bt_leaves::BTFeedback feedback_; //action feedback (SUCCESS, FAILURE)
  robogame_bt_leaves::BTResult result_;//action feedback  (same as feedback for us)


public:


  BTAction(std::string name) :
    as_(nh_, name, boost::bind(&BTAction::executeCB, this, _1), false),
    action_name_(name)
  {
   //Starts the action server
    as_.start();


  }

  ~BTAction(void)
  {


  }

  void executeCB(const robogame_bt_leaves::BTGoalConstPtr &goal)
  {

    // publish info to the console for the user
    ROS_INFO("Starting Action");

    // start executing the action
    int i = 0;
    while(i < 5)
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested())
      {
        ROS_INFO("Action Halted");

        // set the action state to preempted
        as_.setPreempted();
        break;
      }


      ROS_INFO("Executing Action");

      ros::Duration(0.5).sleep();//waiting for 0.5 seconds
      i++;

   }

    if (i == 5)
    {
        setStatus(SUCCESS);
    }

  }


//returns the status to the client (Behavior Tree)
  void setStatus(int status){
      //Set The feedback and result of BT.action
      feedback_.status = status;
      result_.status = feedback_.status;
      // publish the feedback
      as_.publishFeedback(feedback_);
      // setSucceeded means that it has finished the action (it has returned SUCCESS or FAILURE).
      as_.setSucceeded(result_);

      switch(status){//Print for convenience
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


int main(int argc, char** argv)
{
  ros::init(argc, argv, "action");
      ROS_INFO(" Enum: %d",RUNNING);
      ROS_INFO(" Action Ready for Ticks");
  BTAction bt_action(ros::this_node::getName());
  ros::spin();

  return 0;
}
