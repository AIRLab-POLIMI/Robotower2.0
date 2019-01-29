#include <ros/ros.h>
#include "planning2/intentions/intention.h"

using IntentionPlanning::Escape;

class Escape: public Intention{
    private:
        ros::Time startingTime_; 

    public:
        Escape():Intention(){
            startingTime_ = ros::Time::now();
            intentionName_ = "escape";
            code_ = ESCAPE_CODE;
        }
        int getNextIntention();
        void evaluateIntention(bool isSafe);
        void start(int id);
};