#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "map_odom_listener");

	ros::NodeHandle node;
	tf::TransformListener listener;
	ros::Publisher odom_map_AMCL = node.advertise<geometry_msgs::PoseStamped>("map_odom_amcl", 10);   

	// wait for some seconds while the rest of the nodes start running. Avoids error when the
	// novacore boards take longer to start and no tf is present.
	ros::Duration(5).sleep();
	ros::Rate rate(100);
	
	while (node.ok()){

		tf::StampedTransform transform;
			
		try{
			listener.lookupTransform( "/map","/base_link", ros::Time(0), transform);
		}
		catch (tf::TransformException ex){
			ROS_ERROR("From /map_odom_listener: Could not find /map tf! Retrying in 1 second...");
			ros::Duration(1).sleep();
		}

		geometry_msgs::PoseStamped pose;       
		
		pose.pose.orientation.x = transform.getRotation().x();
		pose.pose.orientation.y = transform.getRotation().y();
		pose.pose.orientation.z = transform.getRotation().z();
		pose.pose.orientation.w = transform.getRotation().w();

		pose.pose.position.x = transform.getOrigin().x();
		pose.pose.position.y = transform.getOrigin().y();
		pose.pose.position.z = transform.getOrigin().z();

		odom_map_AMCL.publish(pose);

		rate.sleep();
	}
	return 0;
};

