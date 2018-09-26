cmake_minimum_required(VERSION 2.8.3)
project(tracker_kalman)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
				rospy
				std_msgs
				message_generation)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
## catkin_install_python()


## Generate messages in the 'msg' folder
add_message_files(
   FILES
   PlayerPosition.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  DEPENDS message_runtime
)