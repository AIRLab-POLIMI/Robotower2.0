#include <steering_behavior/steering_behavior.h>
#include <steering_behavior/seek.h>
#include <steering_behavior/stop.h>
#include <steering_behavior/arrival.h>
#include <steering_behavior/flee.h>
#include <steering_behavior/deception.h>

#include "planning/SteeringBehaviorEncoded.h"
#include "planning/locomotion_planning.h"


SteeringBehavior::SteeringBehavior* LocomotionPlanning::SteeringFactory::generateSteeringBehavior(planning::SteeringBehaviorEncoded steering_msg){
	switch(steering_msg.behavior_code){
		case FLEE:
			return new Flee(steering_msg.target);
		case STOP:
			return new Stop(steering_msg.target);
		case ARRIVAL:
			return new Arrival(steering_msg.target);
		case SEEK:
			return new Seek(steering_msg.target);
		case DECEPTION:
			// Accepting multiple targets
			return new Deception(steering_msg.targets);
	}
}

SteeringBehavior::SteeringBehavior* LocomotionPlanning::SteeringFactory::generateSteeringBehavior(int steering_code, geometry_msgs::Point32 target){
	switch(steering_code){
		case FLEE:
			return new Flee(target);
		case STOP:
			return new Stop(target);
		case ARRIVAL:
			return new Arrival(target);
		case SEEK:
			return new Seek(target);
		case DECEPTION:
			return new Deception(target);
	}
}

SteeringBehavior::SteeringBehavior* LocomotionPlanning::SteeringFactory::generateSteeringBehavior(int steering_code, std::vector<geometry_msgs::Point32> targets){
	switch(steering_code){
		case FLEE:
			return new Flee(targets[0]);
		case STOP:
			return new Stop(targets[0]);
		case ARRIVAL:
			return new Arrival(targets[0]);
		case SEEK:
			return new Seek(targets[0]);
		case DECEPTION:
			return new Deception(targets);
	}
}

SteeringBehavior::SteeringBehavior* LocomotionPlanning::SteeringFactory::generateSteeringBehavior(int steering_code){
	switch(steering_code){
		case FLEE:
			return new Flee();
		case STOP:
			return new Stop();
		case ARRIVAL:
			return new Arrival();
		case SEEK:
			return new Seek();
		case DECEPTION:
			return new Deception();
	}
}
