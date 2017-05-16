#include "heartbeat/HeartbeatClient.h"
#include "ros/callback_queue.h"
#include "heartbeat/SetState.h"
#include "heartbeat/RegisterNode.h"
#include "heartbeat/UnregisterNode.h"


void HeartbeatClient::timer_callback(const ros::TimerEvent&) {
	ROS_WARN("Timeout!");
}

void HeartbeatClient::state_callback(const heartbeat::State::ConstPtr& msg) {
	_state_timer.stop();
	_state_timer.start();
	_state = msg->value;
	ROS_DEBUG("Received: %u", msg->value);
}

HeartbeatClient::HeartbeatClient(ros::NodeHandle& nh, float heartbeat_timeout) :
		_nh(nh), _heartbeat_timeout(heartbeat_timeout), _spinner(1, &_callback_queue) {
	ros::TimerOptions timer_ops;
	ros::SubscribeOptions sub_ops;

	ros::param::param<float>("/heartbeat/timeout", _state_timeout, 1.0);
	ros::param::param<std::string>("/heartbeat/heartbeat_topic", _heartbeat_topic, "heartbeat");
	ros::param::param<std::string>("/heartbeat/state_topic", _state_topic, "state");

	timer_ops.autostart = false;
	timer_ops.callback = boost::bind(&HeartbeatClient::timer_callback, this, _1);
	timer_ops.callback_queue = &_callback_queue;
	timer_ops.oneshot = false;
	timer_ops.period = ros::Duration(_state_timeout);
	timer_ops.tracked_object = ros::VoidPtr();
	_state_timer = _nh.createTimer(timer_ops);

	sub_ops.init<heartbeat::State>(_state_topic, 1, boost::bind(&HeartbeatClient::state_callback, this, _1));
	sub_ops.tracked_object = ros::VoidPtr();
	sub_ops.callback_queue = &_callback_queue;
	_state_sub = _nh.subscribe(sub_ops);

	_state_service = _nh.serviceClient<heartbeat::SetState>("/heartbeat/set_state");
	_register_service = _nh.serviceClient<heartbeat::RegisterNode>("/heartbeat/register_node");
	_unregister_service = _nh.serviceClient<heartbeat::UnregisterNode>("/heartbeat/unregister_node");

	if (_heartbeat_timeout != 0) {
		heartbeat::RegisterNode register_node;


		register_node.request.node_name.data = ros::this_node::getName();
		register_node.request.timeout.data = heartbeat_timeout;


		if (!_register_service.call(register_node)) {
			ROS_INFO("Heartbeat register RPC failed");
			return;
		}

		if (register_node.response.success) {
			ros::AdvertiseOptions adv_ops;

			adv_ops.init<heartbeat::Heartbeat>(_heartbeat_topic, 1);
			adv_ops.callback_queue = &_callback_queue;
			_heartbeat_pub = _nh.advertise(adv_ops);
			ROS_INFO("Node heartbeat sucessfully registered");
		} else {
			// TODO: check result, and then?
			ROS_WARN("Node heartbeat failed to register");
		}
	}
}

HeartbeatClient::~HeartbeatClient(void) {
	if (_heartbeat_timeout != 0) {
		heartbeat::UnregisterNode unregister_node;

		unregister_node.request.node_name.data = ros::this_node::getName();

		if (!_unregister_service.call(unregister_node)) {
			ROS_WARN("Heartbeat unregister RPC failed");
			return;
		}

		if (unregister_node.response.success) {
			ROS_INFO("Node unregistered from Heartbeat");
		}
	}

	stop();
}


void HeartbeatClient::start(void) {
	_spinner.start();
	_state_timer.start();
}

void HeartbeatClient::stop(void) {
	_state_timer.stop();
	_spinner.stop();
}

bool HeartbeatClient::setState(heartbeat::State::_value_type to_state) {
	heartbeat::SetState req_state;

	req_state.request.from.value = _state;
	req_state.request.to.value = to_state;

	if (!_state_service.call(req_state)) {
		return false;
	}

	if (req_state.response.current.value != to_state) {
		_state = req_state.response.current.value;
		return false;
	}

	return true;
}

void HeartbeatClient::alive(void) {
	heartbeat::Heartbeat msg;

	msg.node_name.data = ros::this_node::getName();
	_heartbeat_pub.publish(msg);
}
