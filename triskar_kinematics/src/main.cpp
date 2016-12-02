#include <ros/ros.h>
#include <time.h>
#include <signal.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <triskar_kinematics/Velocity.h>
#include<r2p/EncoderStamped.h>
#include <boost/bind.hpp>
#include <cmath>

#define CENTER_DISTANCE  0.5          /* Distance between wheels and 
                                            center of the robot [m] */
#define WHEEL_RADIUS     0.1          /* wheel_radius [m] */ 

ros::Publisher pub;

void mySigintHandler(int sig)
{
  ROS_INFO_STREAM("Shutdown node upon request... Bye!");
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

void encoderReceived(const r2p::EncoderStampedConstPtr &menc1, const r2p::EncoderStampedConstPtr &menc2, const r2p::EncoderStampedConstPtr &menc3){

    // CHECK THIS...
   float _speed[3];                         /* encoder values*/
   _speed[0] = menc1->encoder.delta;
   _speed[1] = menc2->encoder.delta;
   _speed[2] = menc3->encoder.delta;
   
   triskar_kinematics::Velocity velocity;

   const float R = WHEEL_RADIUS;
   const float L = CENTER_DISTANCE;


   float dx = R*sin(M_PI/3.0)*_speed[0] - R*sin(M_PI/3.0)*_speed[1];
   float dy = R*cos(M_PI/3.0)*_speed[0] + R*cos(M_PI/3.0)*_speed[1] - R * _speed[2];
   float dphi = (_speed[0]+_speed[1]+_speed[2])*R/L;

   
    /// PUBLISH THE RESULTS
    velocity.header.stamp = ros::Time::now();
    velocity.linear[0] = dx;
    velocity.linear[1] = dy;
    velocity.angular = dphi;

    pub.publish(velocity);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "TriskarForward_kinematics");
	ros::NodeHandle  nh;
	
	// Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, mySigintHandler);
	
	ROS_INFO_STREAM("Setting up the kinematics node...");
    pub = nh.advertise<triskar_kinematics::Velocity>("/robogame/velocity", 10);
    ROS_INFO_STREAM("Publishing '/robogame/velocity' data...");
    message_filters::Subscriber<r2p::EncoderStamped> enc_sub_1(nh, "/triskar/stamped_encoder1", 1);
	ROS_INFO_STREAM("Subscribed to '/triskar/encoder1' topic...");
    message_filters::Subscriber<r2p::EncoderStamped> enc_sub_2(nh, "/triskar/stamped_encoder2", 1);
	ROS_INFO_STREAM("Subscribed to '/triskar/encoder2' topic...");
    message_filters::Subscriber<r2p::EncoderStamped> enc_sub_3(nh, "/triskar/stamped_encoder3", 1);
	ROS_INFO_STREAM("Subscribed to '/triskar/encoder3' topic...");
	ROS_INFO_STREAM("Registering callback...");
    message_filters::TimeSynchronizer<r2p::EncoderStamped,
                                      r2p::EncoderStamped,
                                      r2p::EncoderStamped> sync(enc_sub_1,enc_sub_2,enc_sub_3, 1000);
    sync.registerCallback(boost::bind(&encoderReceived,_1, _2, _3));
    ROS_INFO_STREAM("DONE...");
    ROS_INFO_STREAM("Running normally...");
	ros::spin();
	return 0;
}
