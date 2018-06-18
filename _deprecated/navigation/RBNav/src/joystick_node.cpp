/* A node that takes joystick (ps3) commands.
	author: Ewerton Lopes (ewerlopes@gmail.com)
*/

#include <ros/ros.h>
#include <cmath>
#include <time.h>
#include <signal.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>
#include <heartbeat/HeartbeatClient.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


// Global tf listener pointer
tf::TransformListener* tfListener;

 class JoyTeleop
 {
	public:
		JoyTeleop();
		ros::NodeHandle nh;

	private:
		void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);
		void navGoalCallback(const geometry_msgs::PointStamped::ConstPtr &msg);
		void moveBaseNavGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
		void updateParameters();
		void timerCallback(const ros::TimerEvent& e);
		void publishZeroMessage();
		geometry_msgs::PointStamped poseStamped_to_pointStamped(geometry_msgs::PoseStamped pose);

		geometry_msgs::PointStamped last_target_position_;
		tf::Vector3 normalized_target_velocity_;
		double linearScale, angularScale;
		double maxLinearScale = 0, maxAngularScale = 0;
		int deadmanButton, linearXAxis, linearYAxis, angularAxis;
		bool canMove;
		bool isExit = false;
		ros::Subscriber joySub_, navGoalSub_, moveBaseNavGoalSub_, pixelPosSub;
		ros::Publisher twistPub;
		ros::Timer timeout;
};

JoyTeleop::JoyTeleop() {
	joySub_ = nh.subscribe("/joy", 10, &JoyTeleop::joyCallback, this);
	navGoalSub_ = nh.subscribe("/nav_goal", 10, &JoyTeleop::navGoalCallback, this);
	moveBaseNavGoalSub_ = nh.subscribe("/move_base_simple/goal", 10, &JoyTeleop::moveBaseNavGoalCallback, this);
	twistPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10); //TODO /unsafe/cmd_vel
	updateParameters();
}

geometry_msgs::PointStamped JoyTeleop::poseStamped_to_pointStamped(geometry_msgs::PoseStamped pose){
	geometry_msgs::PointStamped point;

	point.header = pose.header;
	point.point = pose.pose.position;
	
	ROS_INFO_STREAM("poseStamped_to_pointStamped: point from pose: " << point);

	return point;
}

void JoyTeleop::navGoalCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
	last_target_position_ = *msg;

	ROS_INFO("JoyTeleop::navGoalCallback");
}

void JoyTeleop::moveBaseNavGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	
	last_target_position_ = poseStamped_to_pointStamped(*msg);

	ROS_INFO("JoyTeleop::moveBaseNavGoalCallback");
}

void JoyTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr &msg) {

	if (linearScale >= maxLinearScale){
		linearScale = maxLinearScale;
	}
	
	if (linearScale <= 0){
		linearScale = 0;
	}
	
	if (angularScale >= maxAngularScale){
		angularScale = maxAngularScale;
	}
	
	if (angularScale <= 0){
		angularScale = 0;
	}

	// Send a message to rosout with the details.
	ROS_INFO_STREAM("Receiving joystick input..."	<<
			"\nCurrent LinearScale(x-axis):  "		<< linearScale <<
			"\nCurrent AngularScale(y-axis): "		<< angularScale <<
			"\nCurrent Goal:                 "		<< last_target_position_);

	// process and publish
	geometry_msgs::Twist twistMsg;

	// check deadman switch
	bool switchActive = (msg->buttons[deadmanButton] == 1);

	if (switchActive) {
		if (msg->buttons[3]==1){
			ROS_INFO_STREAM("Increasing linearScale by 0.5\%...");
			linearScale += 0.01;//linearScale * 0.05;
		}else if (msg->buttons[2]==1){
			ROS_INFO_STREAM("Decreasing linearScale by 0.5\%...");
			linearScale -= 0.01;// linearScale * 0.05;
		}else if (msg->buttons[0]==1){
			ROS_INFO_STREAM("Increasing angularScale by 0.5\%...");
			angularScale += 0.01;// angularScale * 0.05;
		}else if (msg->buttons[1]==1){
			ROS_INFO_STREAM("Decreasing linearScale by 0.5\%...");
			angularScale -= 0.01;// angularScale * 0.05;
		}else if (msg->buttons[7]==1){
            ROS_INFO_STREAM("Automatic navigation ON.");
            

			// lookup player transform and compute angular velocity to follow the player
			////////////////////////////////// TODO comment better
	  		tf::StampedTransform playerTransform;
	  		
			float angle_diff = 0;

            try{
				tfListener->waitForTransform("/kinect2_link", ros::Time(0), "/player_link", ros::Time(0), "/map", ros::Duration(0.1));
				tfListener->lookupTransform("/kinect2_link", "/player_link", ros::Time(0), playerTransform);
				angle_diff = atan2( playerTransform.getOrigin().y(), playerTransform.getOrigin().x());
			} catch (tf::TransformException ex) {
				ROS_ERROR("%s",ex.what());
			}

			
			
			if (std::abs(angle_diff) > (5*M_PI/180)){
				twistMsg.angular.z = 4.0 * angle_diff;
				ROS_INFO_STREAM("Threshold activated!");
			} else {
				twistMsg.angular.z = 0.0;
			}
			            
            ROS_INFO_STREAM("Robot<->Player angle mismatch: " << angle_diff << " rad");
            ROS_INFO_STREAM("Angular action threshold: " << (5*M_PI/180) << " rad");
            ROS_INFO_STREAM("Angular value: " << twistMsg.angular.z << " rad");
            

			float x_target_position = last_target_position_.point.x;
			float y_target_position = last_target_position_.point.y;

			// broadcast and lookup the transform of the goal and compute the linear velocity to navigate toward the goal
			////////////////////////////////// TODO comment better
			// TF-Broadcaster
			static tf::TransformBroadcaster br;
			tf::StampedTransform resultGoalTransform;
			tf::Transform frameGoalTransform;

			frameGoalTransform.setOrigin( tf::Vector3(x_target_position, y_target_position, 0) );
			tf::Quaternion q;
			q.setRPY(0, 0, 0);
			frameGoalTransform.setRotation(q);
			ros::Time now = ros::Time::now();
			br.sendTransform(tf::StampedTransform(frameGoalTransform, now, last_target_position_.header.frame_id, "/nav_goal"));
			
			try{
				tfListener->waitForTransform("/base_link", now, "/nav_goal", now, "/map", ros::Duration(0.1));
				tfListener->lookupTransform("/base_link", "/nav_goal", now, resultGoalTransform);

			} catch (tf::TransformException ex) {
				ROS_ERROR("%s",ex.what());
			}
			
			tf::Vector3 relative_goal(resultGoalTransform.getOrigin());

			// float relative_goal_x = resultGoalTransform.getOrigin().x();
			// float relative_goal_y = resultGoalTransform.getOrigin().y();

			ROS_INFO_STREAM("\n\nnot normalized velocity vector:" << resultGoalTransform.getOrigin().x() << ",\t" <<  resultGoalTransform.getOrigin().y());
			// ROS_INFO("\n\nnot normalized velocity vector: %2.3f, %2.3f", relative_goal_x, relative_goal_y);

			tf::Vector3 tmp_target_velocity = tf::Vector3(resultGoalTransform.getOrigin().x(), resultGoalTransform.getOrigin().y(), 0);
			tmp_target_velocity.normalize();
			
			// if distance is greater then the threshold, set the velocity toward the goal, otherwise the goal is considered reached AND THE VELOCITY IS ZERO
			if(std::pow(relative_goal.x(), 2) + std::pow(relative_goal.y(), 2) > std::pow(0.5, 2)){ // no magic allowed
				normalized_target_velocity_ = tmp_target_velocity;
			} else {
				normalized_target_velocity_ = tf::Vector3(0, 0, 0);
			}

			ROS_INFO_STREAM("\n\n    normalized velocity vector:" << normalized_target_velocity_.x() << ",\t" << normalized_target_velocity_.y() << "\n\n");

			twistMsg.linear.x = linearScale*normalized_target_velocity_.x();
			twistMsg.linear.y = linearScale*normalized_target_velocity_.y();

        }else{
			twistMsg.linear.x = linearScale*msg->axes[linearXAxis];
        	twistMsg.linear.y = linearScale*msg->axes[linearYAxis];
            twistMsg.angular.z = angularScale*msg->axes[angularAxis];
			ROS_INFO("twist from axes: l.x: %2.3f, l.x: %2.3f, l.x: %2.3f", twistMsg.linear.x, twistMsg.linear.y, twistMsg.angular.z);
        }

		twistPub.publish(twistMsg);

	} else if (canMove) {
		publishZeroMessage();
	}

	canMove = switchActive;

	// reset the timeout timer
	/*if (timeout) {
		timeout.stop();
	}
	timeout = nh.createTimer(ros::Duration(2), &JoyTeleop::timerCallback, this, true);
	*/
}

void JoyTeleop::updateParameters() {
	// update the parameters for processing the joystick messages
	nh.getParam("max_linear_scale", maxLinearScale);

	nh.getParam("max_angular_scale", maxAngularScale);

	if (!nh.getParam("linear_scale", linearScale))
		linearScale = 1;

	if (!nh.getParam("angular_scale", angularScale))
		angularScale = 0.5;

	if (!nh.getParam("deadman_button", deadmanButton))
		deadmanButton = 5;

	if (!nh.getParam("linear_x_axis", linearXAxis))
		linearXAxis = 1;

	if (!nh.getParam("linear_y_axis", linearYAxis))
		linearYAxis = 0;

	if (!nh.getParam("angular_axis", angularAxis))
		angularAxis = 2;
}

void JoyTeleop::timerCallback(const ros::TimerEvent& e) {
	publishZeroMessage();
}

void JoyTeleop::publishZeroMessage() {
	geometry_msgs::Twist msg;
	msg.linear.x = 0;
	msg.linear.y = 0;
	msg.angular.z = 0;

	twistPub.publish(msg);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "teleop_node");
	JoyTeleop teleop;

	ROS_INFO_STREAM("RBNav initialized");

	// This must be set after the first NodeHandle is created.
	// HeartbeatClient Initialize.
	HeartbeatClient hb(teleop.nh, 0.2);
	hb.start();
	heartbeat::State::_value_type state = heartbeat::State::INIT;
	hb.setState(state);
	
	double maxLinearScale, maxAngularScale;
	
	if (!teleop.nh.getParam("/max_linear_scale",maxLinearScale) || !teleop.nh.getParam("/max_angular_scale", maxAngularScale)){
	 	ROS_FATAL("max_linear_scale and max_angular_scale are required!");
	 	ros::shutdown();
	 	return -1;
	 }

	tfListener = new tf::TransformListener();
  	// Loop at 100Hz until the node is shutdown.
    ros::Rate rate(100);
	// set heartbeat node state to started
    state = heartbeat::State::STARTED;
    bool success = hb.setState(state);
	
	while(ros::ok()){
        // Issue heartbeat.
        hb.alive();
		ros::spinOnce();
        // Wait until it's time for another iteration.
        rate.sleep() ;
    }

    success = hb.setState(heartbeat::State::STOPPED);
    // Issue heartbeat.
    hb.alive();
    hb.stop();
	return 0;
}
