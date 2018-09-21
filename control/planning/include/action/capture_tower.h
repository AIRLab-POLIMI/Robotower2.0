#include "action/action_lib.h"
#include <geometry_msgs/Point32.h>
#include <steering_behavior/steering_behavior.h>
#include <steering_behavior/arrival.h>
#include <steering_behavior/seek.h>

using Action::CaptureTower;

class CaptureTower: public AbstractAction {
	private:
		int tower_target_;
		geometry_msgs::Point32 target_;

		//TODO REMOVE
		bool swap_;
	public:
		CaptureTower(int tower_target):AbstractAction(){
			//target_ = target;
			tower_target_ = tower_target;

    		meaningful_behaviors_.push_back(new Arrival());
			meaningful_behaviors_.push_back(new Seek());
		}
		CaptureTower(int tower_target, geometry_msgs::Point32 target):AbstractAction(){
			target_ = target;
			tower_target_ = tower_target;

    		meaningful_behaviors_.push_back(new Arrival());
			meaningful_behaviors_.push_back(new Seek());
		}
		

		planning::SteeringBehaviorEncoded generateSteeringMsg(int behavior_index);
		void generateTargetPoint();

};
