cmake_minimum_required(VERSION 2.8.3)
project(player_tracker)


find_package(Boost REQUIRED system thread)
include_directories(
  include ${catkin_INCLUDE_DIRS}
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(${Boost_LIBRARY_DIRS})
add_definitions(${Boost_DEFINITIONS})

if(NOT Boost_FOUND)
    message(ERROR " Boost not found!")
endif(NOT Boost_FOUND)


find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})


#########################################################
# FIND OpenMP
#########################################################
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")
endif()

#########################################################
# FIND PCL
#########################################################
find_package( PCL REQUIRED )
include_directories(  ${catkin_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#########################################################
# FIND Eigen
#########################################################
find_package(Eigen3 REQUIRED eigen3)
include_directories(${Eigen3_INCLUDE_DIRS})
link_directories(${Eigen3_LIBRARY_DIRS})
add_definitions(${Eigen3_DEFINITIONS})
if(NOT Eigen3_FOUND)
    message(ERROR "Eigen3 not found!")
endif(NOT Eigen3_FOUND)


#########################################################
# FIND Octomap
#########################################################
# find_package(octomap REQUIRED)
# include_directories(${OCTOMAP_INCLUDE_DIRS})
# link_libraries(${PROJECT_NAME} ${OCTOMAP_LIBRARIES})

set(CMAKE_CXX_FLAGS "-std=c++11 -g -fopenmp")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf
  interactive_markers
  rosbag
  image_geometry
  message_generation
  laser_geometry
  laser_assembler
  costmap_2d
  message_filters
)

add_message_files(
  FILES
  TrackVariance.msg
  Blob.msg
  Person.msg
  PersonArray.msg
  Leg.msg
  LegArray.msg
  PersonEvidence.msg
  PersonEvidenceArray.msg
  Tower.msg
  TowerArray.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   LegProbability.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# catkin_package(
#   CATKIN_DEPENDS message_runtime
# )

catkin_package(
  DEPENDS
    OpenMP
    message_runtime
)


include_directories(
	INCLUDE include
	${catkin_INCLUDE_DIRS}
)


##########################
# detect_leg_clusters
##########################
add_executable(
  detect_leg_clusters
  src/leg_tracker/detect_leg_clusters.cpp
  src/particle_filter/laser_processor.cpp
  src/leg_tracker/cluster_features.cpp
)
target_link_libraries(
  detect_leg_clusters
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

##########################
# local_occupancy_grid_mapping
##########################
add_executable(
  local_occupancy_grid_mapping
  src/leg_tracker/local_occupancy_grid_mapping.cpp
  src/particle_filter/laser_processor.cpp
)
target_link_libraries(
  local_occupancy_grid_mapping
  ${catkin_LIBRARIES}
)

##############################
# player position estimation
#############################
add_executable(
  player_position_estimation
  src/particle_filter/player_position.cpp
)
target_link_libraries(
  player_position_estimation
  ${catkin_LIBRARIES}
)
add_executable(
  player_ground_truth
  src/particle_filter/player_ground_truth.cpp
)
target_link_libraries(
  player_ground_truth
  ${catkin_LIBRARIES}
)


##############################
# distances tracking
#############################
add_executable(
  player_robot_distance
  src/distance_tracking/player_robot_distance.cpp
)
target_link_libraries(
  player_robot_distance
  ${catkin_LIBRARIES}
)
add_executable(
  robot_tower_distance
  src/distance_tracking/robot_tower_distance.cpp
)
target_link_libraries(
  robot_tower_distance
  ${catkin_LIBRARIES}
)


##########################
# extract_positive_training_clusters
##########################
add_executable(
  extract_positive_training_clusters
  src/leg_tracker/extract_positive_training_clusters.cpp
  src/particle_filter/laser_processor.cpp
)

target_link_libraries(
  extract_positive_training_clusters
  ${catkin_LIBRARIES}
)

##########################
# train_leg_detector
##########################
add_executable(
  train_leg_detector
  src/leg_tracker/train_leg_detector.cpp
  src/particle_filter/laser_processor.cpp
  src/leg_tracker/cluster_features.cpp
)

target_link_libraries(
  train_leg_detector
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

##########################
# costmap_translator
##########################
add_executable(
  costmap_translator
  src/particle_filter/costmap_translator.cpp
  # src/particle_filter/costmap_translator_2.cpp
)
target_link_libraries(
  costmap_translator
  ${catkin_LIBRARIES}
)

##########################
# particle_filter
##########################
add_executable(
  particle_filter
  src/particle_filter/Particle.cpp
  src/particle_filter/ParticleFilter.cpp
  src/particle_filter/main.cpp
  src/particle_filter/laser_processor.cpp
)

target_link_libraries(
  particle_filter
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${OpenMP_LIBS}
)


add_dependencies(particle_filter ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(costmap_translator ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(local_occupancy_grid_mapping ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(local_occupancy_grid_mapping ${PROJECT_NAME}_gencfg)
add_dependencies(detect_leg_clusters ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(detect_leg_clusters ${PROJECT_NAME}_gencfg)
add_dependencies(extract_positive_training_clusters ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(extract_positive_training_clusters ${PROJECT_NAME}_gencfg)
add_dependencies(train_leg_detector ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(train_leg_detector ${PROJECT_NAME}_gencfg)

# install(
#   PROGRAMS scripts/joint_leg_tracker.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(
#   PROGRAMS scripts/individual_leg_tracker.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(
#   PROGRAMS scripts/leg_distance.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(
#   PROGRAMS scripts/extractor_bounding_box.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(
#   PROGRAMS scripts/detector.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
