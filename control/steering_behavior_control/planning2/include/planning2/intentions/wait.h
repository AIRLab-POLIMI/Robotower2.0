# include "planning2/intentions/intention.h"

using IntentionPlanning::Wait;

class Wait: public Intention{

    public:
        Wait():Intention(){
            intentionName_ = "wait";
            code_ = WAIT_CODE;
        }
        int getNextIntention();
        void evaluateIntention(bool isSafe);

        void start(int id){
            setId(id);
            statusCode_ = RUNNING;
        }
};