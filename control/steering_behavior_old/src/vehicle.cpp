#include <vector>
#include <string>

#include <ros/ros.h>

#include "steering_behavior/vehicle_model.h"
#include "steering_behavior/steering_behavior.h"
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>


VehicleModel::PointVehicle::PointVehicle(){
    // TODO use ROS parameters or configuration file
    bool on_simulation;
    std::string pose_topic;
    std::string vel_topic;

    if (!nh_.getParam("/steering_behavior_node/vel_topic_sub", vel_topic)){
        ROS_ERROR("STEERING BEHAVIOR MANAGER: could not read 'vel_topic_sub' from rosparam!");
        exit(-1);
    }

    if (!nh_.getParam("/steering_behavior_node/pose_topic", pose_topic)){
        ROS_ERROR("STEERING BEHAVIOR MANAGER: could not read 'vel_topic' from rosparam!");
        exit(-1);
    }

    //pose_sub_ = nh_.subscribe(pose_topic, 1, &PointVehicle::updateCurrentPos, this);
    vel_sub_ = nh_.subscribe(vel_topic, 1, &PointVehicle::updateCurrentVelocity, this);

    if (!nh_.getParam("/steering_behavior_node/mass", mass_) ){
        ROS_ERROR("STEERING BEHAVIOR MANAGER: could not read the parameter 'mass' from rosparam! Check steerning_behavior .yaml file!");
        exit(-1);
    }

    if (!nh_.getParam("/steering_behavior_node/max_speed", max_speed_) ){
        ROS_ERROR("STEERING BEHAVIOR MANAGER: could not read the parameter 'mass' from rosparam! Check steerning_behavior .yaml file!");
        exit(-1);
    }

    if (!nh_.getParam("/steering_behavior_node/max_force", max_force_) ){
        ROS_ERROR("STEERING BEHAVIOR MANAGER: could not read the parameter 'mass' from rosparam! Check steerning_behavior .yaml file!");
        exit(-1);
    }

    // read whether we should run on simulation
    if (!nh_.getParam("/steering_behavior_node/simulation", on_simulation)){
        ROS_ERROR("STEERING BEHAVIOR MANAGER: could not read 'simulation' from rosparam!");
        exit(-1);
    }

    if(on_simulation){
        // We should get the initial position from simulation params
        ROS_INFO("Simulating...");
        std::vector<float> initial_position_array;
        std::vector<float> initial_velocity_array;
        if (!nh_.getParam("/steering_behavior_node/current_pos", initial_position_array)){
            ROS_ERROR("STEERING BEHAVIOR MANAGER: could not read 'current_pos' from rosparam!");
            exit(-1);
        }

        current_pos_.x = initial_position_array[0];
        current_pos_.y = initial_position_array[1];
        current_pos_.z = initial_position_array[2];

        if (!nh_.getParam("/steering_behavior_node/current_vel", initial_velocity_array)){
            ROS_ERROR("STEERING BEHAVIOR MANAGER: could not read 'current_vel' from rosparam!");
            exit(-1);
        }

        current_vel_.x = initial_velocity_array[0];
        current_vel_.y = initial_velocity_array[1];
        current_vel_.z = initial_velocity_array[2];  

    }   

     
}
            
geometry_msgs::Twist VehicleModel::PointVehicle::generateCommandVel(){

    geometry_msgs::Twist command;
    // Calculate desired velocity according to the current steering behavior
    geometry_msgs::Vector3 desired_velocity = steering_behavior_->calculate_desired_velocity(current_pos_);
    ROS_WARN("Desired velocity x:%.2f, y:%.2f, z:%.2f", desired_velocity.x, desired_velocity.y, desired_velocity.z);

    // Truncate the velocity according to vehicle specs
    desired_velocity = SteeringBehavior::VectorUtility::truncate(desired_velocity, max_speed_);


    // Calculate the steering force to apply according to the current steering behavior
    geometry_msgs::Vector3 steering_force = steering_behavior_->calculate_steering_force(current_vel_, desired_velocity);
    ROS_WARN("Steering force x:%.2f, y:%.2f, z:%.2f", steering_force.x, steering_force.y, steering_force.z);

    // Truncate the force according to vehicle specs
    steering_force = SteeringBehavior::VectorUtility::truncate(steering_force, max_force_);

    ROS_WARN("Desired velocity x:%.2f, y:%.2f, z:%.2f", desired_velocity.x, desired_velocity.y, desired_velocity.z);
    ROS_WARN("Steering force x:%.2f, y:%.2f, z:%.2f", steering_force.x, steering_force.y, steering_force.z);

    // Calculate the velocity resulting from applying the steering force to the vehicle
    geometry_msgs::Vector3 acceleration = SteeringBehavior::VectorUtility::scalar_multiply(steering_force, (1/mass_));
    geometry_msgs::Vector3 command_lin = SteeringBehavior::VectorUtility::vector_sum(current_vel_, acceleration);

    // Truncate the velocity according to vehicle specs
    command.linear = SteeringBehavior::VectorUtility::truncate(command_lin, max_speed_);

    // TODO REMOVE THIS LINE AS IT'S JUST FAKE WAITING FOR ROS TO TAKE OVER
    // current_vel_ = command.linear;
    return command;
}

// Updates the current position of the robot
void VehicleModel::PointVehicle::updateCurrentPos(const geometry_msgs::Pose& msg){
    // TODO get robot pose using ROS
    ROS_INFO("Updating Pose...");
    current_pos_.x = msg.position.x;
    current_pos_.y = msg.position.y;
    current_pos_.z = msg.position.z;

}

void VehicleModel::PointVehicle::updateCurrentPos(){
        /**
        Gets robot global position. That is, performs a TF transformation from /base_link to /map and returns
        x,y and theta.
        OUTPUTS:
        @ a 3D-numpy array defined as: [x, y, theta] w.r.t /map.
        **/
        tf::StampedTransform transform;
        try{
            listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
            current_pos_.x = transform.getOrigin().getX();
            current_pos_.y = transform.getOrigin().getY();
            current_pos_.z = transform.getOrigin().getZ();


            double roll, pitch, yaw;
            tf::Quaternion quat = transform.getRotation();            
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            
            current_rotation_wrt_map_ = yaw;
            ROS_INFO("Current rotation: %.2f", current_rotation_wrt_map_);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
}

geometry_msgs::Twist VehicleModel::PointVehicle::alignCommand(geometry_msgs::Twist cmd){
    geometry_msgs::Twist rotated_cmd;
    rotated_cmd.linear = SteeringBehavior::VectorUtility::rotate(cmd.linear, -current_rotation_wrt_map_);
    return rotated_cmd;
}

// Updates current velocity of robot
void VehicleModel::PointVehicle::updateCurrentVelocity(const geometry_msgs::Twist& msg){
    // TODO get robot current velocity using ROS
     // TODO GET ROTATION BETWEEN BASE_LINK E MAP
    ROS_ERROR("Unrotated_vel x:%.2f, y:%.2f", msg.linear.x, msg.linear.y);
    current_vel_ = SteeringBehavior::VectorUtility::rotate(msg.linear, current_rotation_wrt_map_);
    ROS_ERROR("Rotated_vel x:%.2f, y:%.2f", current_vel_.x, current_vel_.y);
}
