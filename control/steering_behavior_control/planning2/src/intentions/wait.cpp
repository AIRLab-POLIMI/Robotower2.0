#include "planning2/intentions/wait.h"

int IntentionPlanning::Wait::getNextIntention(){
    return PLAY_GAME_CODE;
}

void IntentionPlanning::Wait::evaluateIntention(bool isSafe){}