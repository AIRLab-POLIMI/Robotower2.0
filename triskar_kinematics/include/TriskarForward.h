/* COPYRIGHT (c) 2016 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#pragma once
#include <ros/ros.h>
#include <time.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <triskar_kinematics/Velocity.h>
#include<r2p/EncoderStamped.h>
#include <boost/bind.hpp>

class TriskarForward{
	public:
		TriskarForward();

	private:
		void encoderReceived(const r2p::EncoderStamped& menc1, 
		              const r2p::EncoderStamped& menc2,
		              const r2p::EncoderStamped& menc3);
        void Configure();
		
		message_filters::Subscriber<r2p::EncoderStamped>* enc_sub_1;
        message_filters::Subscriber<r2p::EncoderStamped>* enc_sub_2;
   		message_filters::Subscriber<r2p::EncoderStamped>* enc_sub_3;
        message_filters::TimeSynchronizer<r2p::EncoderStamped,
                                          r2p::EncoderStamped,
                                          r2p::EncoderStamped>* sync;
		ros::Publisher  pub;
		ros::NodeHandle  nh;
        
        float _speed[3];                /* encoder values*/
        float center_distance;          /* Distance between wheels and 
                                           center of the robot [m] */
        float wheel_radius;             /* wheel_radius [m] */ 
};
