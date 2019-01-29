#ifndef INTENTION_H
#define INTENTION_H

#include "planning2/Intention.h"

#include <string>

#define CONTINUE -1

#define PLAY_GAME_CODE 0
#define ESCAPE_CODE 1
#define WAIT_CODE 2

#define RUNNING 0
#define COMPLETED 1
#define ABORTED 2 

#define ESCAPE_TIMEOUT 3

namespace IntentionPlanning{
    class Intention{
        protected:
            std::string intentionName_;
            int code_;
            int id_;

            int statusCode_;

            int proposedNextIntentionCode_;

            bool isCompleted_;
            bool isAborted_;

        public:
            Intention(){
                statusCode_ = RUNNING;
                // isCompleted_ = false;
                // isAborted_ = false;
            }
            std::string getName(){
                return intentionName_;
            }

            int getId(){
                return id_;
            }

            void setId(int id){
                id_ = id;
            }

            int getCode(){
                return code_;
            }

            bool isRunning(){
                return statusCode_ == RUNNING;
            }

            bool isCompleted(){

                return statusCode_ == COMPLETED;
            }

            bool isAborted(){
                // if(statusCode_ == ABORTED){
                //     return true;
                // }
                return statusCode_ == ABORTED;
            }


            planning2::Intention generateIntentionMessage(){
                planning2::Intention msg;

                msg.id = getId();
                msg.intention_code = getCode();
                msg.intention_name = getName();

                return msg;
            }


            // Returns the code of the next intention if it has changed, otherwise returns NO_CHANGE value
            virtual int getNextIntention() = 0;
            // Evaluates the intention given the current state of the game
            virtual void evaluateIntention(bool isSafe) = 0;

            virtual void start(int id) = 0;//{
            //     setId(id);
            //     isCompleted_ = false;
            //     isAborted_ = false;
            // } 
    };

    class PlayGame;
    class Escape;
    class Wait;
}
#endif