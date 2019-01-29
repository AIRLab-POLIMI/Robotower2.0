#include "planning2/intentions/play_game.h"

int IntentionPlanning::PlayGame:: getNextIntention(){
    if(isAborted()){
        return ESCAPE_CODE;
    }
    else{
        return CONTINUE;
    }
}

void IntentionPlanning::PlayGame::evaluateIntention(bool isSafe){
    // Check if we're in a safe situation
    if(!isSafe){
        // If not, mark the intention as aborted
        statusCode_ = ABORTED;
    }
}