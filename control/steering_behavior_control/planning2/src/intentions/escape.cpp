#include "planning2/intentions/escape.h"

int IntentionPlanning::Escape::getNextIntention(){
    if(isCompleted()){
        return PLAY_GAME_CODE;
    }
    else if(isAborted()){
        return WAIT_CODE;
    }
    else{
        return CONTINUE;
    }

}

void IntentionPlanning::Escape::evaluateIntention(bool isSafe){
    // Check if we're in a safe situation
    if(isSafe){
        // If so, mark the intention as completed
        statusCode_ = COMPLETED;
        proposedNextIntentionCode_ = PLAY_GAME_CODE;
    }
    else{
        // If we're not, check the timeout
        ros::Duration elapsedTime = ros::Time::now() - startingTime_;
        if(elapsedTime.toSec() > ESCAPE_TIMEOUT){
            // If it has exprired, mark the intention as aborted
            statusCode_ = ABORTED;
        }
    }
    
}

void IntentionPlanning::Escape::start(int id){
    setId(id);
    startingTime_ = ros::Time::now();
    statusCode_ = RUNNING;
}