#include "steering_behavior/steering_behavior.h"
#include "steering_behavior/seek.h"
#include "steering_behavior/stop.h"
#include "steering_behavior/arrival.h"
#include "steering_behavior/flee.h"

SteeringBehavior::SteeringBehavior* SteeringBehavior::SteeringFactory::generateSteeringBehavior(int steering_code, geometry_msgs::Point32 target){
	switch(steering_code){
		case FLEE:
			return new Flee(target);
		case STOP:
			return new Stop(target);
		case ARRIVAL:
			return new Arrival(target);
		case SEEK:
			return new Seek(target);
	}
}

SteeringBehavior::SteeringBehavior* SteeringBehavior::SteeringFactory::generateSteeringBehavior(int steering_code){
	switch(steering_code){
		case FLEE:
			return new Flee();
		case STOP:
			return new Stop();
		case ARRIVAL:
			return new Arrival();
		case SEEK:
			return new Seek();
	}
}
