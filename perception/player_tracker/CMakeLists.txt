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
)

add_message_files(
  FILES
  TrackVariance.msg
  Blob.msg
  Cluster.msg
  ClusterArray.msg
  Person.msg
  PersonArray.msg
  Leg.msg
  LegArray.msg
  PersonEvidence.msg
  PersonEvidenceArray.msg
  TowerArray.msg
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
  INCLUDE_DIRS include
  LIBRARIES player_tracker
  DEPENDS
    OpenMP
    message_runtime
)


include_directories(
	INCLUDE include
	${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/laser_processor.cpp
)

##########################
# detect_leg_clusters
##########################
add_executable(
  detect_leg_clusters
  src/detect_leg_clusters.cpp
  src/laser_processor.cpp
  src/cluster_features.cpp
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
  src/local_occupancy_grid_mapping.cpp
  src/laser_processor.cpp
)
target_link_libraries(
  local_occupancy_grid_mapping
  ${catkin_LIBRARIES}
)

##########################
# test_local_occupancy_grid_mapping
##########################
add_executable(
  test_local_occupancy_grid_mapping
  src/test_local_occupancy_grid_mapping.cpp
  src/laser_processor.cpp
)
target_link_libraries(
  test_local_occupancy_grid_mapping
  ${catkin_LIBRARIES}
)

##########################
# extract_positive_training_clusters
##########################
add_executable(
  extract_positive_training_clusters
  src/extract_positive_training_clusters.cpp
  src/laser_processor.cpp
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
  src/train_leg_detector.cpp
  src/laser_processor.cpp
  src/cluster_features.cpp
)

target_link_libraries(
  train_leg_detector
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

##########################
# particle_filter
##########################
add_executable(
  particle_filter
  src/particle_filter/Particle.cpp
  src/particle_filter/ParticleFilter.cpp
  src/particle_filter/MultipleParticleFilter.cpp
  src/particle_filter/pf_tracker.cpp
)

target_link_libraries(
  particle_filter
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${OpenMP_LIBS}
)
# target_compile_options(particle_filter PRIVATE ${OpenMP_FLAGS})
# add_dependencies(particle_filter ${catkin_EXPORTED_TARGETS})

##########################
# blob_publisher
##########################
add_executable(
  blob_publisher
  src/particle_filter/blob_publisher.cpp
  src/particle_filter/BlobPublisher.cpp
)

target_link_libraries(
  blob_publisher
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)


##########################
# motion_detector
##########################
# add_executable(
#     motion_detector
#     #     src/spatial_temporal.cpp
# )

# target_link_libraries(
#   motion_detector
#   ${catkin_LIBRARIES}
#   ${Eigen3_LIBRARIES}
#   ${PCL_LIBRARIES}
#   ${OpenMP_LIBS}
# )
# target_compile_options(motion_detector PRIVATE ${OpenMP_FLAGS})
# add_dependencies(motion_detector ${catkin_EXPORTED_TARGETS})


##########################
# spatial_temporal_node
##########################
add_executable(
  spatial_temporal_node
  src/spatial_temporal_node.cpp
  src/spatial_temporal.cpp
)

target_link_libraries(
  spatial_temporal_node
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenMP_LIBS}
)
target_compile_options(spatial_temporal_node PRIVATE ${OpenMP_FLAGS})
add_dependencies(spatial_temporal_node ${catkin_EXPORTED_TARGETS})

##########################
# cloud_cluster
##########################
add_executable(
    cloud_cluster
    src/cloud_cluster.cpp
)

target_link_libraries(
  cloud_cluster
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

##########################
# laser_cluster_extractor
##########################
add_executable(
    laser_cluster_extractor
    src/laser_cluster_extractor.cpp
    src/laser_processor.cpp
)

target_link_libraries(
  laser_cluster_extractor
  ${catkin_LIBRARIES}
)

add_dependencies(test_local_occupancy_grid_mapping ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(local_occupancy_grid_mapping ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(local_occupancy_grid_mapping ${PROJECT_NAME}_gencfg)
add_dependencies(detect_leg_clusters ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(detect_leg_clusters ${PROJECT_NAME}_gencfg)
add_dependencies(extract_positive_training_clusters ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(extract_positive_training_clusters ${PROJECT_NAME}_gencfg)
add_dependencies(train_leg_detector ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(train_leg_detector ${PROJECT_NAME}_gencfg)
add_dependencies(particle_filter ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(spatial_temporal_node ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(test_local_occupancy_grid_mapping ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(cloud_cluster ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(blob_publisher ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(laser_cluster_extractor ${PROJECT_NAME}_generate_messages_cpp)

install(
  PROGRAMS scripts/joint_leg_tracker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  PROGRAMS scripts/individual_leg_tracker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  PROGRAMS scripts/leg_distance.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  PROGRAMS scripts/extractor_bounding_box.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  PROGRAMS scripts/detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
