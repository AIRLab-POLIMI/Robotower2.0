#include "planning2/vehicles/vehicle.h"

VehicleRepresentation::PointVehicle::PointVehicle(){
    currentMaxSpeed_ = 0.75;
    currentMass_ = 7.0;
    currentMaxForce_ = 5.0;

    velSub_ = nh_.subscribe("vel", 1, &VehicleRepresentation::PointVehicle::velCallback, this);
}

void VehicleRepresentation::PointVehicle::setBehavior(LocomotionPlanning::SteeringBehavior* behavior){
    currentBehavior_ = behavior;
    updateWeightsMap_ = currentBehavior_ -> getUpdateWeightsMap();
    currentBehavior_ -> setMaxSpeed(currentMaxSpeed_ * updateWeightsMap_.at("speed"));
    currentBehavior_ -> setSlowingRadius(1.0);
}

void VehicleRepresentation::PointVehicle::initUpdateMap(){
    updateWeightsMap_.insert({"speed", 1.0});
    updateWeightsMap_.insert({"mass", 1.0});
}
            
void VehicleRepresentation::PointVehicle::locateVehicle(geometry_msgs::Point32 currentPos, float currentRotation){
    currentPos_ = currentPos;
    currentRotationWrtMap_ = currentRotation;
}

void VehicleRepresentation::PointVehicle::velCallback(geometry_msgs::Twist velocity){
    currentVel_ = LocomotionPlanning::VectorUtility::rotate(velocity.linear, currentRotationWrtMap_);
}
            
geometry_msgs::Twist VehicleRepresentation::PointVehicle::generateCommandVel(){
    geometry_msgs::Twist command;
    // Calculate desired velocity according to the current steering behavior
    geometry_msgs::Vector3 desiredVelocity = currentBehavior_->calculate_desired_velocity(currentPos_);
    
    // Truncate the velocity according to vehicle specs
    desiredVelocity = LocomotionPlanning::VectorUtility::truncate(desiredVelocity, currentMaxSpeed_ * updateWeightsMap_.at("speed"));

    // Calculate the steering force to apply according to the current steering behavior
    geometry_msgs::Vector3 steeringForce = currentBehavior_->calculate_steering_force(currentVel_, desiredVelocity);
    
    // Truncate the force according to vehicle specs
    steeringForce = LocomotionPlanning::VectorUtility::truncate(steeringForce, currentMaxForce_);

    // Calculate the velocity resulting from applying the steering force to the vehicle
    geometry_msgs::Vector3 acceleration;

    acceleration = LocomotionPlanning::VectorUtility::scalar_multiply(steeringForce, (1/(currentMass_ * updateWeightsMap_.at("mass"))));
    
    geometry_msgs::Vector3 command_lin = LocomotionPlanning::VectorUtility::vector_sum(currentVel_, acceleration);

    // Truncate the velocity according to vehicle specs
    command.linear = LocomotionPlanning::VectorUtility::truncate(command_lin, currentMaxSpeed_ * updateWeightsMap_.at("speed"));

    return alignWithMap(command);
}

geometry_msgs::Twist VehicleRepresentation::PointVehicle::alignWithMap(geometry_msgs::Twist cmd){
    geometry_msgs::Twist rotatedCmd;
    rotatedCmd.linear = LocomotionPlanning::VectorUtility::rotate(cmd.linear, -currentRotationWrtMap_);
    return rotatedCmd;
}