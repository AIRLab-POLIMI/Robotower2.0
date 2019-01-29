#include <ros/ros.h>
#include "planning2/intentions/intention.h"
#include <planning/SafetyMsg.h>

namespace IntentionPlanning{

    class IntentionPlanner{
        private:
            ros::NodeHandle nh_;
            ros::Publisher intentionPub_;
            ros::Subscriber gameStateSub_;
            ros::Subscriber safetySub_;

            std::map <int, Intention*> intentions_;
            Intention* currentIntention_;

            int currentId_;
            bool isSafe_;

            void safetyCallback(planning::SafetyMsg sMsg);


        public:
            IntentionPlanner();
            void loop();
    };
}