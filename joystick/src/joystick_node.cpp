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
#include <heartbeat/HeartbeatClient.h>


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

	// Send a message to rosout with the details.
	ROS_INFO_STREAM("Receiving joystick input..."  <<
			"\nCurrent LinearScale(x-axis):"       <<
			 linearScale			               <<
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
		}else if (msg->buttons[7]==1){
            ROS_INFO_STREAM("Automatic rotation ON.");
            
	  		tf::StampedTransform playerTransform;
	  		
            try{
				tfListener->waitForTransform("/kinect2_link", ros::Time(0), "/player_link", ros::Time(0), "/map", ros::Duration(1.0));
				tfListener->lookupTransform("/kinect2_link", "/player_link", ros::Time(0), playerTransform);
			} catch (tf::TransformException ex) {
				ROS_ERROR("%s",ex.what());
			}
		
			float angle_diff = atan2( playerTransform.getOrigin().y(), playerTransform.getOrigin().x());
			
			if (std::abs(angle_diff) > (5*M_PI/180)){
				twistMsg.angular.z = 4.0 * angle_diff;
				ROS_INFO_STREAM("Threshold activated!");
			}
			            
            ROS_INFO_STREAM("Robot<->Player angle mismatch: " << angle_diff << " rad");
            ROS_INFO_STREAM("Angular action threshold: " << (5*M_PI/180) << " rad");
            ROS_INFO_STREAM("Angular value: " << twistMsg.angular.z << " rad");
            
        }else{
            twistMsg.angular.z = angularScale*msg->axes[angularAxis];
        }

        twistMsg.linear.x = linearScale*msg->axes[linearXAxis];
        twistMsg.linear.y = linearScale*msg->axes[linearYAxis];
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
	ros::init(argc, argv, "teleop_node");
	JoyTeleop teleop;

	// This must be set after the first NodeHandle is created.
	// HeartbeatClient Initialize.
	HeartbeatClient hb(teleop.nh, 0.2);
	hb.start();
	heartbeat::State::_value_type state = heartbeat::State::INIT;
	hb.setState(state);

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
