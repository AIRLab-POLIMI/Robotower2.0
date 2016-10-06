#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

 class JoyTeleop
 {
	public:
		JoyTeleop();

	private:
		void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);
		void updateParameters();
		void timerCallback(const ros::TimerEvent& e);
		void publishZeroMessage();

		double linearScale, angularScale;
		int deadmanButton, linearXAxis, linearYAxis, angularAxis;
		bool canMove;
		ros::Subscriber joySub;
		ros::Publisher twistPub;
		ros::NodeHandle nh;
		ros::Timer timeout;
};

JoyTeleop::JoyTeleop() {
	joySub = nh.subscribe("/joy", 10, &JoyTeleop::joyCallback, this);
	twistPub = nh.advertise<geometry_msgs::Twist>("spacenav/twist", 10);

	updateParameters();
}

void JoyTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr &msg) {

	// Send a message to rosout with the details.
	ROS_INFO_STREAM("Receiving joystick input..."    <<
			        "\nCurrent LinearScale(x-axis):" << linearScale <<
			        "\nCurrent AngularScale(y-axis):"<< angularScale);

	// process and publish
	geometry_msgs::Twist twistMsg;

	// check deadman switch
	bool switchActive = (msg->buttons[deadmanButton] == 1);

	if (switchActive) {
		if (msg->buttons[3]==1){
			ROS_INFO_STREAM("Increasing linearScale by 0.5\%...");
			linearScale += linearScale * 0.05;
		}else if (msg->buttons[2]==1){
			ROS_INFO_STREAM("Decreasing linearScale by 0.5\%...");
			linearScale -= linearScale * 0.05;
		}else if (msg->buttons[0]==1){
			ROS_INFO_STREAM("Increasing angularScale by 0.5\%...");
			angularScale += angularScale * 0.05;
		}else if (msg->buttons[1]==1){
			ROS_INFO_STREAM("Decreasing linearScale by 0.5\%...");
			angularScale -= angularScale * 0.05;
		}
		twistMsg.linear.x = linearScale*msg->axes[linearXAxis];
		twistMsg.linear.y = linearScale*msg->axes[linearYAxis];
		twistMsg.angular.z = angularScale*msg->axes[angularAxis];

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
	if (!nh.getParam("linear_scale", linearScale))
		linearScale = 1;

	if (!nh.getParam("angular_scale", angularScale))
		angularScale = 0.5;

	if (!nh.getParam("deadman_button", deadmanButton))
		deadmanButton = 5;

	if (!nh.getParam("linear_axis", linearXAxis))
		linearXAxis = 1;

	if (!nh.getParam("linear_axis", linearYAxis))
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
	ros::init(argc, argv, "triskar_joy_node");
	JoyTeleop joy_teleop_node;

	ros::spin();

	return 0;
}
