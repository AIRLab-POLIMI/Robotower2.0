#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>

#define PLAYER_WIN 1
#define TIE 0
#define ROBOT_WIN -1
#define INVALID -100
#define ATTACK_WINDOW 2

namespace Microgame{

    class Microgame{
        public:
            Microgame();

            void start();
            void reset();
            int getWinner();
            void run();
        private:
            ros::NodeHandle nh_;
            ros::Subscriber startAttackSub_;
            ros::Subscriber endAttackSub_;
            ros::Subscriber pressTimeSub_;
            ros::Subscriber resetSub_;

            ros::Publisher gameOutcomePub_;

            bool attacking_;
            int last_attack_target_;
            bool first_attack_;

            void startAttackCallback(std_msgs::Int8 startAttackMsg);
            void endAttackCallback(std_msgs::Int8 message);
            void evaluateAttackOutcome();

            void pressTimeCallback(std_msgs::Float64 pressTimeMsg);
            void resetCallback(std_msgs::Bool reset);

            float chargingTime_;

            ros::Time gameStartTime_;
            double gameDuration_;

            bool hasProgressed_;
            int playerScore_;
            int robotScore_;

            int attackNumber_;
    };
}