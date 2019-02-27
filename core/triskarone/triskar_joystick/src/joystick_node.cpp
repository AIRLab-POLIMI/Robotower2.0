/* A node that takes joystick (ps3) commands.
	author: Ewerton Lopes (ewerlopes@gmail.com)
*/

#include <ros/ros.h>
#include <cmath>
#include <time.h>
#include <signal.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Joy.h>


// Global tf listener pointer
tf::TransformListener* tfListener;

 class JoyTeleop
 {
	public:
		JoyTeleop();
		ros::NodeHandle nh;

	private:
		void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);
		void updateParameters();
		void timerCallback(const ros::TimerEvent& e);
		void publishZeroMessage();

		double linearScale, angularScale;
		double maxLinearScale = 0, maxAngularScale = 0;
		int deadmanButton, linearXAxis, linearYAxis, angularAxis;
		bool canMove;
		bool isExit = false;
		ros::Subscriber joySub;
        ros::Subscriber pixelPosSub;
		ros::Publisher twistPub;
		ros::Timer timeout;
};

JoyTeleop::JoyTeleop() {
	joySub = nh.subscribe("/joy", 10, &JoyTeleop::joyCallback, this);
	twistPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	updateParameters();
}

void JoyTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr &msg) {

	// process and publish
	geometry_msgs::Twist twistMsg;

	if (msg->buttons[deadmanButton]) {		// if deadman switch is pressed
		if (msg->buttons[3]){
			ROS_DEBUG_STREAM("Increasing linearScale by 0.5\%...");
			linearScale += 0.01;//linearScale * 0.05;
		}else if (msg->buttons[2]){
			ROS_DEBUG_STREAM("Decreasing linearScale by 0.5\%...");
			linearScale -= 0.01;// linearScale * 0.05;
		}else if (msg->buttons[0]){
			ROS_DEBUG_STREAM("Increasing angularScale by 0.5\%...");
			angularScale += 0.01;// angularScale * 0.05;
		}else if (msg->buttons[1]){
			ROS_DEBUG_STREAM("Decreasing linearScale by 0.5\%...");
			angularScale -= 0.01;// angularScale * 0.05;
        }else{
            twistMsg.angular.z = angularScale*msg->axes[angularAxis];
        }

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
		
        twistMsg.linear.x = linearScale*msg->axes[linearXAxis];
        twistMsg.linear.y = linearScale*msg->axes[linearYAxis];
		twistPub.publish(twistMsg);

	}else{
		publishZeroMessage();
	}

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
	msg.angular.z = 0;
	twistPub.publish(msg);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "teleop_node");
	JoyTeleop teleop;
	
	double maxLinearScale, maxAngularScale;
	
	if (!teleop.nh.getParam("/max_linear_scale",maxLinearScale) || !teleop.nh.getParam("/max_angular_scale", maxAngularScale)){
	 	ROS_FATAL("max_linear_scale and max_angular_scale are required!");
	 	ros::shutdown();
	 	return -1;
	 }

	tfListener = new tf::TransformListener();


	ros::spin();

	return 0;
}
