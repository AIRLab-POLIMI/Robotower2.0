#ifndef HEARTBEAT_CLIENT_H
#define HEARTBEAT_CLIENT_H

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "heartbeat/State.h"
#include "heartbeat/Heartbeat.h"

class HeartbeatClient {
private:
	ros::NodeHandle& _nh;
	ros::CallbackQueue _callback_queue;
	ros::AsyncSpinner _spinner;
	ros::Subscriber _state_sub;
	ros::Publisher _heartbeat_pub;
	ros::ServiceClient _state_service;
	ros::ServiceClient _register_service;
	ros::ServiceClient _unregister_service;
	ros::Timer _state_timer;
	heartbeat::State::_value_type _state;
	std::string _state_topic;
	std::string _heartbeat_topic;
	float _state_timeout;
	float _heartbeat_timeout;

	void timer_callback(const ros::TimerEvent&);
	void state_callback(const heartbeat::State::ConstPtr& msg);
public:
	HeartbeatClient(ros::NodeHandle& nh, float heartbeat_timeout = 0);
	~HeartbeatClient(void);
	void start(void);
	void stop(void);
	bool setState(heartbeat::State::_value_type state);
	heartbeat::State::_value_type getState(void);
	void alive(void);
};

inline
heartbeat::State::_value_type HeartbeatClient::getState(void) {
	return _state;
}

#endif /* HEARTBEAT_CLIENT_H */
