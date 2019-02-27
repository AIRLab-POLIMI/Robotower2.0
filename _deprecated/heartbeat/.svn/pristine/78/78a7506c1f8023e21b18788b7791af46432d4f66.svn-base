#include "heartbeat/HeartbeatServer.h"
#include "ros/callback_queue.h"
#include "heartbeat/Heartbeat.h"
#include "heartbeat/State.h"
#include "heartbeat/SetState.h"

void HeartbeatServer::heartbeat_callback(
		const heartbeat::Heartbeat::ConstPtr& msg) {
	std::map<std::string, ros::Timer>::iterator it;

	it = _registered_nodes.find(msg->node_name.data);

	if (it != _registered_nodes.end()) {
		ros::Timer& timer = it->second;
		timer.stop();
		timer.start();
	}

	ROS_DEBUG("Heartbeat from node %s", msg->node_name.data.c_str());
}

void HeartbeatServer::heartbeat_timeout(const ros::TimerEvent&) {
	_state = heartbeat::State::HALT;
	spin();
	ROS_ERROR("Heartbeat timeout!");
}

bool HeartbeatServer::set_state(heartbeat::SetState::Request &req,
		heartbeat::SetState::Response &res) {

	/* Check if current state is consistent */
	if (req.from.value != _state) {
		ROS_DEBUG("State transition %u -> %u rejected: current state is %u",
				req.from.value, req.to.value, _state);
		return false;
	}

	/* Transition to same state */
	if (req.to.value == _state) {
		return true;
	}

	/* Transition to HALT is always allowed */
	if (req.to.value == heartbeat::State::HALT) {
		_state = req.to.value;
	}

	switch (_state) {
	case heartbeat::State::HALT:
		/* from HALT only ->SAFE is allowed */
		if (req.to.value == heartbeat::State::SAFE) {
			_state = req.to.value;
		}
		break;

	case heartbeat::State::MANUAL:
		/* from MANUAL everything but ->AUTO is allowed */
		if (req.to.value != heartbeat::State::AUTO) {
			_state = req.to.value;
		}
		break;

	case heartbeat::State::SAFE:
		/* from SAFE everything is allowed */
		_state = req.to.value;
		break;

	case heartbeat::State::ASSISTED:
		/* from ASSISTED everything but ->AUTO is allowed */
		if (req.to.value != heartbeat::State::AUTO) {
			_state = req.to.value;
		}
		break;

	case heartbeat::State::AUTO:
		/* from AUTO only ->SAFE are allowed */
		if (req.to.value == heartbeat::State::SAFE) {
			_state = req.to.value;
		}
		break;

	default:
		ROS_DEBUG("Invalid state: %u", req.to.value);
	}

	res.current.value = _state;

	if (_state == req.to.value) {
		spin();
		ROS_INFO("State updated: %u -> %u", req.from.value, _state);
		return true;
	} else {
		ROS_WARN("State transition not allowed: %u -> %u", req.from.value,
				req.to.value);
		return false;
	}
}

bool HeartbeatServer::register_node(heartbeat::RegisterNode::Request &req,
		heartbeat::RegisterNode::Response &res) {
	ros::TimerOptions timer_ops;

	/* No mutex here, server requests are serialized - to be checked. */
	if (_registered_nodes.find(req.node_name.data) != _registered_nodes.end()) {
		if (_registered_nodes.erase(req.node_name.data)) {
			ROS_INFO("Node replaced: %s", req.node_name.data.c_str());
		} else {
			ROS_INFO("Node unregister failed: %s", req.node_name.data.c_str());
			res.success = false;
		}
	}

	timer_ops.autostart = false;
	timer_ops.callback = boost::bind(&HeartbeatServer::heartbeat_timeout, this,
			_1);
	timer_ops.callback_queue = &_callback_queue;
	timer_ops.oneshot = false;
	timer_ops.period = ros::Duration(req.timeout.data);
	timer_ops.tracked_object = ros::VoidPtr();
	ros::Timer timer = _nh.createTimer(timer_ops);

	_registered_nodes.insert(
			std::pair<std::string, ros::Timer>(req.node_name.data, timer));
	res.success = true;
	ROS_INFO("Node registered: %s", req.node_name.data.c_str());

	return true;
}

bool HeartbeatServer::unregister_node(heartbeat::UnregisterNode::Request &req,
		heartbeat::UnregisterNode::Response &res) {

	if (_registered_nodes.erase(req.node_name.data)) {
		ROS_INFO("Node unregistered: %s", req.node_name.data.c_str());
		res.success = true;
	} else {
		ROS_INFO("Node unregister failed: %s", req.node_name.data.c_str());
		res.success = false;
	}
	return true;
}

HeartbeatServer::HeartbeatServer(ros::NodeHandle & nh) :
		_nh(nh), _spinner(10, &_callback_queue) {
	ros::TimerOptions timer_ops;
	ros::AdvertiseOptions adv_ops;
	ros::SubscribeOptions sub_ops;

	ros::param::param<float>("/heartbeat/state_update_period", _state_period,
			1.0);

	sub_ops.init<heartbeat::Heartbeat>("heartbeat", 100,
			boost::bind(&HeartbeatServer::heartbeat_callback, this, _1));
	sub_ops.tracked_object = ros::VoidPtr();
	sub_ops.callback_queue = &_callback_queue;
	_heartbeat_sub = _nh.subscribe(sub_ops);

	_state_pub = _nh.advertise<heartbeat::State>("state", 10);

	_set_state_service = _nh.advertiseService("/heartbeat/set_state",
			&HeartbeatServer::set_state, this);
	_register_node_service = _nh.advertiseService("/heartbeat/register_node",
			&HeartbeatServer::register_node, this);
	_unregister_node_service = _nh.advertiseService(
			"/heartbeat/unregister_node", &HeartbeatServer::unregister_node,
			this);

	_state = heartbeat::State::HALT; /* TODO: rosparam */
}

HeartbeatServer::~HeartbeatServer(void) {
	stop();
}

void HeartbeatServer::start(void) {
	_spinner.start();
}

void HeartbeatServer::stop(void) {
	_spinner.stop();
}

void HeartbeatServer::spin(void) {
	heartbeat::State msg;

	msg.value = _state;
	_state_pub.publish(msg);
}
