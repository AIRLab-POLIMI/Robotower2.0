/* COPYRIGHT (c) 2016 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#include "TriskarForward.h"
#include <boost/bind.hpp>
#include <cmath>

TriskarForward::TriskarForward(){

    pub = nh.advertise<triskar_kinematics::Velocity>("/robogame/velocity", 10);
    enc_sub_1 = new message_filters::Subscriber<r2p::EncoderStamped>(nh, "/triskar/encoder1", 1000);
    enc_sub_2 = new message_filters::Subscriber<r2p::EncoderStamped>(nh, "/triskar/encoder2", 1000);
    enc_sub_3 = new message_filters::Subscriber<r2p::EncoderStamped>(nh, "/triskar/encoder3", 1000);
    sync = new message_filters::TimeSynchronizer<r2p::EncoderStamped,
                                                 r2p::EncoderStamped,
                                                 r2p::EncoderStamped>(*enc_sub_1,*enc_sub_2,*enc_sub_3, 1000);
    //sync->registerCallback(boost::bind(&TriskarForward::encoderReceived, this, _1, _2, _3));
	
	/* Sets the Forward kinematics node configuration*/
	center_distance = 0.5; // <- SET THIS
	wheel_radius = 0.1;   //  <- SET THIS
}

void TriskarForward::encoderReceived(const r2p::EncoderStamped& menc1, const r2p::EncoderStamped& menc2, 
const r2p::EncoderStamped& menc3){

    // CHECK THIS...
   _speed[0] = menc1.encoder.delta;
   _speed[1] = menc2.encoder.delta;
   _speed[2] = menc3.encoder.delta;
   
   triskar_kinematics::Velocity velocity;

   const float R = this->wheel_radius;
   const float L = this->center_distance;


   float dx = R*sin(M_PI/3.0)*_speed[0] - R*sin(M_PI/3.0)*_speed[1];
   float dy = R*cos(M_PI/3.0)*_speed[0] + R*cos(M_PI/3.0)*_speed[1] - R * _speed[2];
   float dphi = (_speed[0]+_speed[1]+_speed[2])*R/L;

   
    /// PUBLISH THE RESULTS
    velocity.linear[0] = dx;
    velocity.linear[1] = dy;
    velocity.angular = dphi;

    pub.publish(velocity);
}
