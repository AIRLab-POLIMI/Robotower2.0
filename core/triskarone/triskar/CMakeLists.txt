cmake_minimum_required(VERSION 2.8.3)
project(triskar)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs triskar_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

###########
## Build ##
###########


add_library(triskar src/PixyPublisher)
add_dependencies(${PROJECT_NAME} triskar_msgs_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(teleop_pixy_joy src/PixyTeleop.cpp)
add_dependencies(teleop_pixy_joy triskar_msgs_generate_messages_cpp triskar_msgs_gencpp triskar_msgs_EXPORTED_TARGETS ${catkin_EXPORTED_TARGETS})
target_link_libraries(teleop_pixy_joy triskar ${catkin_LIBRARIES})

add_executable(pixy_demo src/PixyDemo.cpp)
add_dependencies(pixy_demo triskar_msgs_generate_messages_cpp triskar_msgs_gencpp triskar_msgs_EXPORTED_TARGETS ${catkin_EXPORTED_TARGETS})
target_link_libraries(pixy_demo triskar ${catkin_LIBRARIES})

add_executable(random_exploration src/RandomExploration.cpp)
add_dependencies(random_exploration triskar_msgs_generate_messages_cpp triskar_msgs_gencpp triskar_msgs_EXPORTED_TARGETS ${catkin_EXPORTED_TARGETS})
target_link_libraries(random_exploration ${catkin_LIBRARIES})


#############
## Install ##
#############
 
install(TARGETS teleop_pixy_joy
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
#install(FILES
#   # myfile1
#   # myfile2
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
