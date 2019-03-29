#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>

class PlayerPositionRepublisher{

    private:
        ros::NodeHandle nh_;
        ros::Subscriber optitrackSubscriber_;
        ros::Publisher playerPositionPublisher_; 
        geometry_msgs::Pose2D position2D_;

        void optitrackCallback(geometry_msgs::Pose2D position2D){
            position2D_ = position2D;
        }


        
    public:

    PlayerPositionRepublisher(){
        optitrackSubscriber_ = nh_.subscribe("/player/ground_pose", 1, &PlayerPositionRepublisher::optitrackCallback, this);
        playerPositionPublisher_ = nh_.advertise<geometry_msgs::PointStamped>("/player_ground_truth", 1000);

    }

    void publishPosition(){
         geometry_msgs::PointStamped playerPosition;
            playerPosition.header.stamp = ros::Time::now();
            playerPosition.header.frame_id = "/map";
            playerPosition.point.y = (position2D_.x + 1.615);
            playerPosition.point.x = -(position2D_.y - 1.72);
            playerPositionPublisher_.publish(playerPosition);
    }
        

};

int main(int argc, char **argv) {
	ros::init(argc, argv, "player_ground_truth");

	ros::NodeHandle nh;
    ros::Rate r(20);
    PlayerPositionRepublisher repub;
    while(ros::ok()){
        repub.publishPosition();
        ros::spinOnce();
        r.sleep();
    }
	return 0;
}