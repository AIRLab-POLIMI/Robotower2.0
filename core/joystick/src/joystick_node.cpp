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
		void unsafeCmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);
		void updateParameters();
		void timerCallback(const ros::TimerEvent& e);
		void publishZeroMessage();

		geometry_msgs::Twist lastUnsafeTwistMsg_;

		double linearScale, angularScale;
		double maxLinearScale = 0, maxAngularScale = 0;
		int deadmanButton, linearXAxis, linearYAxis, angularAxis;
		bool canMove;
		bool isExit = false;
		ros::Subscriber joySub;
		ros::Subscriber unsafeCmdVelSub;
        ros::Subscriber pixelPosSub;
		ros::Publisher twistPub;
		ros::Timer timeout;
		ros::Time unsafe_cmd_vel_time;
};

JoyTeleop::JoyTeleop() {
	joySub = nh.subscribe("/joy", 10, &JoyTeleop::joyCallback, this);
	unsafeCmdVelSub = nh.subscribe("/unsafe/cmd_vel", 10, &JoyTeleop::unsafeCmdVelCallback, this);
	twistPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	updateParameters();
}

void JoyTeleop::unsafeCmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
	unsafe_cmd_vel_time = ros::Time::now();
	lastUnsafeTwistMsg_ = *msg;
}

void JoyTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr &msg) {

	// process and publish
	geometry_msgs::Twist twistMsg;

	if (msg->buttons[4]){							// if autonomous
		// let /unsafe/cmd_vel be published on /cmd_vel

		ros::Time now = ros::Time::now();
		ros::Duration time_diff = unsafe_cmd_vel_time - now;
		if (time_diff.toSec() < 0.5){
			twistMsg = lastUnsafeTwistMsg_;
			twistPub.publish(twistMsg);
		}else{
			ROS_WARN("unsafe/cmd_vel too old... skipping..");
		}
	}else if (msg->buttons[deadmanButton]) {		// if deadman switch is pressed
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
		}else if (msg->buttons[7]){
            ROS_DEBUG_STREAM("Automatic rotation ON.");
            
	  		tf::StampedTransform playerTransform;
	  		float angle_diff = 0;

            try{
				tfListener->waitForTransform("/kinect2_link", ros::Time(0), "/player_link", ros::Time(0), "/map", ros::Duration(1.0));
				tfListener->lookupTransform("/kinect2_link", "/player_link", ros::Time(0), playerTransform);
				angle_diff = atan2( playerTransform.getOrigin().y(), playerTransform.getOrigin().x());
			} catch (tf::TransformException ex) {
				ROS_ERROR("%s",ex.what());
				angle_diff = 0;
			}
		
			
			if (std::abs(angle_diff) > (5*M_PI/180)){
				twistMsg.angular.z = 4.0 * angle_diff;
				ROS_DEBUG_STREAM("Threshold activated!");
			} else {
				twistMsg.angular.z = 0.0;
			}
			            
            ROS_DEBUG_STREAM("Robot<->Player angle mismatch: " << angle_diff << " rad");
            ROS_DEBUG_STREAM("Angular action threshold: " << (5*M_PI/180) << " rad");
            ROS_DEBUG_STREAM("Angular value: " << twistMsg.angular.z << " rad");
            
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
