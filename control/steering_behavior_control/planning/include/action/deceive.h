#include "action/action_lib.h"
#include <geometry_msgs/Point32.h>
#include <steering_behavior/deception.h>

using Action::Deceive;

class Deceive: public AbstractAction {
	private:
		int tower_target_;
		geometry_msgs::Point32 target_;
        std::vector<geometry_msgs::Point32> targets_;

		//TODO REMOVE
		bool swap_;
	public:
		// Deceive(int tower_target):AbstractAction(){
		// 	//target_ = target;
		// 	tower_target_ = tower_target;

    	// 	meaningful_behaviors_.push_back(new Arrival());
		// 	meaningful_behaviors_.push_back(new Seek());
		// }
		// Deceive(int tower_target, geometry_msgs::Point32 target):AbstractAction(){
		// 	target_ = target;
		// 	tower_target_ = tower_target;

    	// 	meaningful_behaviors_.push_back(new Arrival());
		// 	meaningful_behaviors_.push_back(new Seek());
		// }
        Deceive(std::vector<geometry_msgs::Point32> targets):AbstractAction(){
			targets_ = targets;
    		meaningful_behaviors_.push_back(new Deception());
		}
		

		planning::SteeringBehaviorEncoded generateSteeringMsg(int behavior_index);
		void generateTargetPoint();

};