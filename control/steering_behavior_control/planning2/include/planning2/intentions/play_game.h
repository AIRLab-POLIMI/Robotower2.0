# include "planning2/intentions/intention.h"

using IntentionPlanning::PlayGame;

class PlayGame: public Intention{

    public:
        PlayGame():Intention(){
            intentionName_ = "play_game";
            code_ = PLAY_GAME_CODE;
        }
        int getNextIntention();
        void evaluateIntention(bool isSafe);

        void start(int id){
            setId(id);
            statusCode_ = RUNNING;
        }
};