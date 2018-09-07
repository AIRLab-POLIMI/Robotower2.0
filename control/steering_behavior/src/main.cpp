#include <string>

#include <ros/ros.h>
#include "steering_behavior/flee.h"
#include "steering_behavior/seek.h"
#include "steering_behavior/stop.h"
#include "steering_behavior/arrival.h"
#include "steering_behavior/vehicle_model.h"
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>

#define FLEE 0
#define SEEK 1
#define STOP 2
#define ARRIVAL 3


std::vector<float> target_array;
geometry_msgs::Point32 target;
ros::Publisher t_pub;
ros::Publisher marker_pub;
int count;
bool on_simulation;

geometry_msgs::Point32 generateTarget(){
    geometry_msgs::Point32 target;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    marker.type = visualization_msgs::Marker::SPHERE;
    
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.0;
    
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    if(on_simulation){
        target.x = target_array[0];
        target.y = target_array[1];
        target.z = target_array[2];

    }
    else{
        // TODO generate target for steering behavior
        // Analyze the obstacles around the robot to detect a feasible target
        target.x = 5;
        target.y = 5;
        target.z = 0.0;
    } 

    marker.pose.position.x = target.x;
    marker.pose.position.y = target.y;
    marker.pose.position.z = target.z;

    t_pub.publish(target);
    marker_pub.publish(marker);

    return target; 
}

SteeringBehavior::SteeringBehavior* generateSteeringBehavior(int code){
    // TODO
    switch(code){
        case FLEE:
            return new Flee(generateTarget());
            // break;
        case SEEK:
            return new Seek(generateTarget());
            // break;
        case STOP:
            return new Stop(generateTarget());
            // break;
        case ARRIVAL:
            ROS_WARN("Setting arrival behavior");
            return new Arrival(generateTarget());

    }
}

void changeBehavior(VehicleModel::PointVehicle* vehicle){
    // ROS_ERROR("COUNT: %d", count);
    // if(count == 20){
    //     SteeringBehavior::SteeringBehavior *behavior = generateSteeringBehavior(ARRIVAL);
    //     vehicle->setSteeringBehavior(behavior);    
    // }
}


int main(int argc, char** argv){
    ros::init(argc, argv, "steering_behavior_node");
    ros::NodeHandle nh;
    ros::Rate rate(60);
    std::string vel_topic;
    int behavior_code = STOP;

    if (!nh.getParam("/steering_behavior_node/vel_topic_pub", vel_topic)){
        ROS_ERROR("STEERING BEHAVIOR MANAGER: could not read 'vel_topic_pub' from rosparam!");
        exit(-1);
    }

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>(vel_topic, 1);
    marker_pub = nh.advertise<visualization_msgs::Marker>("/target_steering", 1);

    VehicleModel::PointVehicle *vehicle = new VehicleModel::PointVehicle();

    // read whether we should run on simulation
    if (!nh.getParam("/steering_behavior_node/simulation", on_simulation)){
        ROS_ERROR("STEERING BEHAVIOR MANAGER: could not read 'simulation' from rosparam!");
        exit(-1);
    }

    std::vector<float> initial_velocity_array;
    geometry_msgs::Twist initial_vel;

    if (!nh.getParam("/steering_behavior_node/current_vel", initial_velocity_array)){
        ROS_ERROR("STEERING BEHAVIOR MANAGER: could not read 'current_vel' from rosparam!");
        exit(-1);
    }

    initial_vel.linear.x = initial_velocity_array[0];
    initial_vel.linear.y = initial_velocity_array[1];
    initial_vel.linear.z = initial_velocity_array[2];


    if(on_simulation){
        if (!nh.getParam("/steering_behavior_node/target", target_array)){
            ROS_ERROR("Could not read 'target' from rosparam!");
            exit(-1);
        }

        if (!nh.getParam("/steering_behavior_node/behavior", behavior_code)){
            ROS_ERROR("Could not read 'target' from rosparam!");
            exit(-1);
        }

        t_pub = nh.advertise<geometry_msgs::Point32>("/turtle1/target_point", 10); 
    }
    
    SteeringBehavior::SteeringBehavior *behavior = generateSteeringBehavior(behavior_code);
    vehicle->setSteeringBehavior(behavior);   
	while(ros::ok()){
        generateTarget();
        vehicle->updateCurrentPos();
        // START MOVING IN A DIRECTION BEFORE STEERING -- TESTING PURPOSES
        
        geometry_msgs::Twist cmd = vehicle->generateCommandVel();
        geometry_msgs::Twist rotated_cmd = vehicle->alignCommand(cmd);
        vel_pub.publish(rotated_cmd);

        geometry_msgs::Vector3 linear_command = cmd.linear;
        geometry_msgs::Vector3 linear_command_rot = rotated_cmd.linear;
          
        ROS_INFO("Generated velocity x:%.2f, y:%.2f, z:%.2f", linear_command.x, linear_command.y, linear_command.z);
        ROS_INFO("Generated velocity rotated x:%.2f, y:%.2f, z:%.2f", linear_command_rot.x, linear_command_rot.y, linear_command_rot.z);

        rate.sleep();
		ros::spinOnce();
    }
    return 0;
}
