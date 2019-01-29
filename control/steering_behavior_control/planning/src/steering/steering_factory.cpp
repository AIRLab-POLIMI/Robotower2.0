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
			// Needs to know which tower is the target
			return new Arrival(steering_msg.target, steering_msg.tower_target);
		case SEEK:
			// Needs to know which tower is the target
			return new Seek(steering_msg.target, steering_msg.tower_target);
		case DECEPTION:
			// Accepting multiple targets
			return new Deception(steering_msg.targets, steering_msg.tower_target);
	}
}
