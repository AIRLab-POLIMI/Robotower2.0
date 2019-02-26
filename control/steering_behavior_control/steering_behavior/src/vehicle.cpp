#include <vector>
#include <string>

#include <ros/ros.h>

#include "steering_behavior/vehicle_model.h"
#include "steering_behavior/steering_behavior.h"
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>

#include "steering_behavior/KinematicUpdate.h"

#define MIN_PLAYER_PARAM 0.0
#define MAX_PLAYER_PARAM 5.0
#define MAX_PERTURBATION 0.1

VehicleModel::PointVehicle::PointVehicle(){
    // TODO use ROS parameters or configuration file
    bool on_simulation;
    std::string pose_topic;
    std::string vel_topic;

    if (!nh_.getParam("/planning/vel_topic_sub", vel_topic)){
        ROS_ERROR("VEHICLE: could not read 'vel_topic_sub' from rosparam!");
        exit(-1);
    }

    vel_sub_ = nh_.subscribe(vel_topic, 1, &PointVehicle::updateCurrentVelocity, this);
    difficulty_sub_ = nh_.subscribe("/parameter_id", 1, &PointVehicle::difficultyCallback, this);

    /////////////////////////
    // READING MASS PARAMS
    /////////////////////////
    if (!nh_.getParam("/steering_behavior_node/min_mass", min_mass_) ){
        ROS_ERROR("VEHICLE: could not read the parameter 'min_mass' from rosparam! Check steerning_behavior .yaml file!");
        exit(-1);
    }

    if (!nh_.getParam("/steering_behavior_node/base_mass", base_mass_) ){
        ROS_ERROR("VEHICLE: could not read the parameter 'base_mass' from rosparam! Check steerning_behavior .yaml file!");
        exit(-1);
    }

    if (!nh_.getParam("/steering_behavior_node/max_mass", max_mass_) ){
        ROS_ERROR("VEHICLE: could not read the parameter 'max_mass' from rosparam! Check steerning_behavior .yaml file!");
        exit(-1);
    }

    /////////////////////////
    // READING SPEED PARAMS
    /////////////////////////
    if (!nh_.getParam("/steering_behavior_node/min_speed", min_speed_) ){
        ROS_ERROR("VEHICLE: could not read the parameter 'min_speed' from rosparam! Check steerning_behavior .yaml file!");
        exit(-1);
    }

    if (!nh_.getParam("/steering_behavior_node/base_speed", base_speed_) ){
        ROS_ERROR("VEHICLE: could not read the parameter 'base_speed' from rosparam! Check steerning_behavior .yaml file!");
        exit(-1);
    }
    

    if (!nh_.getParam("/steering_behavior_node/max_speed", max_speed_) ){
        ROS_ERROR("VEHICLE: could not read the parameter 'max_speed' from rosparam! Check steerning_behavior .yaml file!");
        exit(-1);
    }

    /////////////////////////
    // READING FORCE PARAMS
    /////////////////////////

    if (!nh_.getParam("/steering_behavior_node/min_force", min_force_) ){
        ROS_ERROR("VEHICLE: could not read the parameter 'min_force' from rosparam! Check steerning_behavior .yaml file!");
        exit(-1);
    }
    if (!nh_.getParam("/steering_behavior_node/base_force", base_force_) ){
        ROS_ERROR("VEHICLE: could not read the parameter 'base_force' from rosparam! Check steerning_behavior .yaml file!");
        exit(-1);
    }
    if (!nh_.getParam("/steering_behavior_node/max_force", max_force_) ){
        ROS_ERROR("VEHICLE: could not read the parameter 'max_force' from rosparam! Check steerning_behavior .yaml file!");
        exit(-1);
    }

    if (!nh_.getParam("/steering_behavior_node/slowing_radius", slowing_radius_) ){
        ROS_ERROR("VEHICLE: could not read the parameter 'slowing_radius' from rosparam! Check steerning_behavior .yaml file!");
        exit(-1);
    }

    if (!nh_.getParam("/steering_behavior_node/deception_target_change_distance", deception_target_change_distance_) ){
        ROS_ERROR("VEHICLE: could not read the parameter 'deception_target_change_distance' from rosparam! Check steerning_behavior .yaml file!");
        exit(-1);
    }

    // read whether we should run on simulation
    if (!nh_.getParam("/simulation", on_simulation)){
        ROS_ERROR("VEHICLE: could not read 'simulation' from rosparam!");
        exit(-1);
    }
    initDifficulties();
    setDifficulty(1);

    // current_speed_ = base_speed_;
    // current_mass_ = base_mass_;
    current_force_ = base_force_;

    baseline_speed_ = base_speed_;

    is_custom_ = false;
    custom_mass_ = base_mass_;

    scan_sub_ = nh_.subscribe("/scan", 1, &VehicleModel::PointVehicle::laserCallback, this);
    kinematic_update_pub_ = nh_.advertise<steering_behavior::KinematicUpdate>("kinematic_update", 1);

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

void VehicleModel::PointVehicle::initDifficulties(){
    std::map<std::string, float> easy;
    easy.insert({SPEED_KEY, min_speed_});
    easy.insert({MASS_KEY, max_mass_});

    std::map<std::string, float> medium;
    medium.insert({SPEED_KEY, base_speed_});
    medium.insert({MASS_KEY, base_mass_});

    std::map<std::string, float> hard;
    hard.insert({SPEED_KEY, max_speed_});
    hard.insert({MASS_KEY, min_mass_});

    difficulties_.push_back(easy);
    difficulties_.push_back(medium);
    difficulties_.push_back(hard);
}

void VehicleModel::PointVehicle::setDifficulty(int level){
    current_speed_ = difficulties_[level].at(SPEED_KEY);
    current_mass_ = difficulties_[level].at(MASS_KEY);
    ROS_INFO_STREAM("Setting current speed " << current_speed_);
}

void VehicleModel::PointVehicle::difficultyCallback(std_msgs::Int8 difficulty){
    setDifficulty(difficulty.data);
}
            
geometry_msgs::Twist VehicleModel::PointVehicle::generateCommandVel(){

    geometry_msgs::Twist command;
    // Calculate desired velocity according to the current steering behavior
    geometry_msgs::Vector3 desired_velocity = steering_behavior_->calculate_desired_velocity(current_pos_);
    // ROS_WARN("Desired velocity CALULATED x:%.2f, y:%.2f", desired_velocity.x, desired_velocity.y);

    // Truncate the velocity according to vehicle specs
    desired_velocity = SteeringBehavior::VectorUtility::truncate(desired_velocity, current_speed_);
    // ROS_WARN("Desired velocity TRUNCATED x:%.2f, y:%.2f", desired_velocity.x, desired_velocity.y);

    // Calculate the steering force to apply according to the current steering behavior
    geometry_msgs::Vector3 steering_force = steering_behavior_->calculate_steering_force(current_vel_, desired_velocity);
    // ROS_WARN("Steering force CALCULATED x:%.2f, y:%.2f", steering_force.x, steering_force.y);

    // Truncate the force according to vehicle specs
    steering_force = SteeringBehavior::VectorUtility::truncate(steering_force, current_force_);
    // ROS_WARN("Steering force TRUNCATED x:%.2f, y:%.2f", steering_force.x, steering_force.y);

    // Calculate the velocity resulting from applying the steering force to the vehicle
    geometry_msgs::Vector3 acceleration;
    if(is_custom_){
        acceleration = SteeringBehavior::VectorUtility::scalar_multiply(steering_force, (1/custom_mass_));
    }
    else{
        acceleration = SteeringBehavior::VectorUtility::scalar_multiply(steering_force, (1/current_mass_));
    }
    // ROS_WARN("Acceleration x:%.2f, y:%.2f", acceleration.x, acceleration.y);
    
    geometry_msgs::Vector3 command_lin = SteeringBehavior::VectorUtility::vector_sum(current_vel_, acceleration);
    // ROS_WARN("Command CALCULATED x:%.2f, y:%.2f", command_lin.x, command_lin.y);

    // Truncate the velocity according to vehicle specs
    command.linear = SteeringBehavior::VectorUtility::truncate(command_lin, max_speed_);
    // ROS_WARN("Command TRUNCATED x:%.2f, y:%.2f", command.linear.x, command.linear.y);

    if(steering_behavior_->updateTarget(current_scan_, current_pos_, current_rotation_wrt_map_)){
        changeParams(steering_behavior_->getUpdateWeights());
    }
    return command;
}

// Updates the current position of the robot
void VehicleModel::PointVehicle::updateCurrentPos(const geometry_msgs::Pose& msg){
    // TODO get robot pose using ROS
    // ROS_INFO("Updating Pose...");
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
            // ROS_INFO("Current rotation: %.2f", current_rotation_wrt_map_);
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
    // Updates the current velocity considered wrt map
    current_vel_ = SteeringBehavior::VectorUtility::rotate(msg.linear, current_rotation_wrt_map_);
}

void VehicleModel::PointVehicle::changeParams(std::vector<float> update_weights){
    // Changes params to change steering effect
    is_custom_ = true;
    custom_mass_ = custom_mass_ * update_weights[0];
    // current_speed_ = base_speed_ * update_weights[1];
    current_force_ = base_force_ * update_weights[2];

	if(steering_behavior_ != nullptr){
        steering_behavior_ -> setSlowingRadius(getSlowingRadius());
        if(current_speed_ * update_weights[1] > 1.05){
            steering_behavior_ -> setMaxSpeed(1.05);
        }
        else{
            steering_behavior_ -> setMaxSpeed(current_speed_ * update_weights[1]);
        }
        steering_behavior_ -> setDeceptionTargetDistance(getDeceptionChangeTargetDistance());
    }

    ROS_INFO("NEW PARAMS [mass = %.2f, max_speed = %.2f, max_force = %.2f]", current_mass_, current_speed_ * update_weights[1], current_force_);
}

void VehicleModel::PointVehicle::updateKinematicProperties(activity_monitor::PlayerModel model){
    steering_behavior::KinematicUpdate msg;
    float model_domain = MAX_PLAYER_PARAM - MIN_PLAYER_PARAM;
    float model_percentage = (model.cumulative_hyperparam - MIN_PLAYER_PARAM) / model_domain;

    ROS_INFO("Receiving a new estimate param:%.2f, press:%.2f", model.cumulative_hyperparam, model.average_led_per_press);

    float new_speed;
    float new_mass;

    msg.old_baseline = baseline_speed_;
    msg.old_mass = current_mass_;
    msg.old_speed = current_speed_;

    float baseline_update_percentage;

    float new_baseline_speed = min_speed_ + (max_speed_ - min_speed_) * model.expertise;
    float perturbation = MAX_PERTURBATION * (model.last_led_per_press - 1);

    baseline_speed_ = (baseline_speed_ + new_baseline_speed) / 2.0;
    current_speed_ = baseline_speed_ + perturbation;

    // if(model.average_led_per_press > 1){
    //     baseline_update_percentage = model.average_led_per_press - 1;
    //     float room_for_update = (max_speed_ - baseline_speed_);
    //     float update_magnitude = room_for_update * baseline_update_percentage;
    //     float new_baseline = baseline_speed_ + update_magnitude;
    //     baseline_speed_ = new_baseline;
    // }
    // else{
    //     baseline_update_percentage = (1 - model.average_led_per_press);
    //     float room_for_update = (baseline_speed_ - min_speed_);
    //     float update_magnitude = room_for_update * baseline_update_percentage;
    //     float new_baseline = baseline_speed_ - update_magnitude;
    //     baseline_speed_ = new_baseline;
    // }

    // current_speed_ = baseline_speed_;


    msg.current_baseline = baseline_speed_;
    msg.current_mass = current_mass_;
    msg.current_speed = current_speed_;

    kinematic_update_pub_.publish(msg);

    ROS_INFO("NEW SPEED %.2f", current_speed_);
    ROS_INFO("NEW MASS %.2f", current_mass_);

}


void VehicleModel::PointVehicle::resetParams(){
    is_custom_ = false;
    custom_mass_ = base_mass_;
    current_mass_ = base_mass_;
    current_speed_ = base_speed_;
    current_force_ = base_force_;
    baseline_speed_ = base_speed_;

    steering_behavior::KinematicUpdate msg;
    msg.current_baseline = baseline_speed_;
    msg.current_mass = current_mass_;
    msg.current_speed = current_speed_;
    kinematic_update_pub_.publish(msg);
}

void VehicleModel::PointVehicle::initBehavior(){
    if( steering_behavior_ != nullptr){
        is_custom_ = false;
        custom_mass_ = base_mass_;
        steering_behavior_ -> setSlowingRadius(getSlowingRadius());
        steering_behavior_ -> setMaxSpeed(current_speed_);
        steering_behavior_ -> setDeceptionTargetDistance(getDeceptionChangeTargetDistance());
    }
}
