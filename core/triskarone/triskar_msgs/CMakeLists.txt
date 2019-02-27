cmake_minimum_required(VERSION 2.8.3)
project(triskar_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation)



#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   Proximity.msg
   Pixy.msg
   PixyServo.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
)


catkin_package(CATKIN_DEPENDS message_runtime)
