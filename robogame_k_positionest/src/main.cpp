// This program subscribes to wiimote and publishes the interested button state.
#include<ros/ros.h>
#include<ros/console.h>
#include<robogame_k_positionest/estimated_position.h>
#include<std_msgs/Int32.h>
#include <time.h>
#include<iostream>
#include<armadillo>
#define ARMA_USE_CXX11

// Create a publisher object for button state.
ros::Publisher pub;

/** KALMAN FILTER VARIABLES **/
arma::mat x = arma::zeros(2,1);             // initial state (location and velocity)
arma::mat P = { {1000,0}, {0,1000} };       // initial uncertainty
arma::mat u = arma::zeros(2,1);             // external motion
arma::mat F = {{1,1},{0,1}};                // next state function
arma::mat H = {1,0};                        // measurement function
arma::mat R = {0.001};                          // measurement uncertainty
arma::mat I = {{1,0},{0,1}};                // identity matrix
/****************************////

// A callback function . Executed each time a wiimote message arrives
void estimate_position(const std_msgs::Int32 &msg){

    double measurement = (double) msg.data;
    // MEASUREMENT UPDATE
    arma::mat Z = {measurement};
    arma::mat Y = Z.t() - H * x;
    arma::mat S = H * P * H.t() + R;
    arma::mat K = P * H.t() * S.i();
    x = x + (K*Y);
    //P = (I - K*H) * P;
    P = P - K * (R+(K*P*H.t())) * K.t();

    // prediction
    x = F * x + u;
    P = F * P * F.t();

    // publish calculated value
    robogame_k_positionest::estimated_position newmsg;
    newmsg.header.stamp = ros::Time::now();
	newmsg.estimation_position = x[0];
    pub.publish(newmsg);
}

int main (int argc, char** argv){
	// Initialize the ROS system and become a node .
	ros::init(argc, argv, "robogame_position_estimator");
	ros::NodeHandle nh;

	// Create a subscriber object.
	ros::Subscriber sub = nh.subscribe("robogame/player_x_position",
                                        1000, &estimate_position);
	pub = nh.advertise<robogame_k_positionest::estimated_position>("robogame/player_est_position",1000);				// advertise LED and Buzzer state

	ROS_INFO("Kalman estimator initialized!");
	// Let ROS take over .
	ros::spin();
}
