#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>//actionlib
#include <robogame_bt_leaves/BTAction.h>//Definition of action. see /scr/action

enum Status {RUNNING,SUCCESS, FAILURE};


class BTAction
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<robogame_bt_leaves::BTAction> as_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  robogame_bt_leaves::BTFeedback feedback_;
  robogame_bt_leaves::BTResult result_;



public:

  BTAction(std::string name) :
    as_(nh_, name, boost::bind(&BTAction::executeCB, this, _1), false),
    action_name_(name)
  {
 //start the action server (action in sense of Actionlib not BT action)
    as_.start();
     ROS_INFO("Condition Server Started");


  }

  ~BTAction(void)
  {

  }
  void executeCB(const robogame_bt_leaves::BTGoalConstPtr &goal)
  {
    if(true){
    setStatus(SUCCESS);
    }else{
    setStatus(FAILURE);
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
          ROS_INFO("Condition %s Succeeded", ros::this_node::getName().c_str() );
          break;
        case FAILURE:
            ROS_INFO("Condition %s Failed", ros::this_node::getName().c_str() );
          break;
        default:
          break;
        }
    }
  };

int main(int argc, char** argv)
{
    ros::init(argc, argv, "condition");
      ROS_INFO(" Enum: %d",RUNNING);
      ROS_INFO(" condition Ready for Ticks");
  BTAction bt_action(ros::this_node::getName());
  ros::spin();

  return 0;
}
