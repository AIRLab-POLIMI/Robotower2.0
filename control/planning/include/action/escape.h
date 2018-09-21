#include "action/action_lib.h"
#include <geometry_msgs/Point32.h>
#include <steering_behavior/steering_behavior.h>
#include <steering_behavior/flee.h>

using Action::Escape;

class Escape: public AbstractAction {
	private:
		int tower_target_;
		geometry_msgs::Point32 target_;
	public:
		Escape():AbstractAction(){
    		meaningful_behaviors_.push_back(new Flee());
		}
		Escape(geometry_msgs::Point32 target):AbstractAction(){
			target_ = target;

    		meaningful_behaviors_.push_back(new Flee());
		}
		

		planning::SteeringBehaviorEncoded generateSteeringMsg(int behavior_index);
		void generateTargetPoint();

};