#include "BehaviorTree.h"

using namespace RobogameBT;

int main(int argc, char **argv){

	ros::init(argc,argv,"BehaviorTree");
	try
	{
		ROSAction* action = new ROSAction("action");
        ROSCondition* condition = new ROSCondition("condition");


        SequenceNode* sequence1 = new SequenceNode("seq1");

        sequence1->AddChild(condition);
        sequence1->AddChild(action);

        Execute(sequence1, TickPeriod_milliseconds); //from BehaviorTree.cpp

}
    catch (BehaviorTreeException& Exception)
    {
        std::cout << Exception.what() << std::endl;
    }

	return 0;
}
