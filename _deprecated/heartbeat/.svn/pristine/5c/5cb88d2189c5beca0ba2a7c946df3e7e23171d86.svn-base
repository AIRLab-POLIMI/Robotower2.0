#ifndef HEARTBEAT_SERVER_H
#define HEARTBEAT_SERVER_H

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "heartbeat/Heartbeat.h"
#include "heartbeat/State.h"
#include "heartbeat/SetState.h"
#include "heartbeat/RegisterNode.h"
#include "heartbeat/UnregisterNode.h"

class HeartbeatServer {
private:
	ros::NodeHandle& _nh;
	ros::CallbackQueue _callback_queue;
	ros::AsyncSpinner _spinner;
	ros::Publisher _state_pub;
	ros::Subscriber _heartbeat_sub;
	ros::ServiceServer _set_state_service;
	ros::ServiceServer _register_node_service;
	ros::ServiceServer _unregister_node_service;
	heartbeat::State::_value_type _state;
	float _state_period;
	std::map<std::string, ros::Timer> _registered_nodes;

	void heartbeat_callback(const heartbeat::Heartbeat::ConstPtr& msg);
	void heartbeat_timeout(const ros::TimerEvent&);
	void state_timer_callback(const heartbeat::State::ConstPtr& msg);
	bool set_state(heartbeat::SetState::Request &req,
			heartbeat::SetState::Response &res);
	bool register_node(heartbeat::RegisterNode::Request &req,
			heartbeat::RegisterNode::Response &res);
	bool unregister_node(heartbeat::UnregisterNode::Request &req,
			heartbeat::UnregisterNode::Response &res);
public:
	HeartbeatServer(ros::NodeHandle& nh);
	~HeartbeatServer(void);
	void start(void);
	void stop(void);
	void spin(void);
};

#endif /* HEARTBEAT_SERVER_H */
