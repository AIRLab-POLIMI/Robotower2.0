cmake_minimum_required(VERSION 2.8.3)
project(r2p)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation std_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  Encoder.msg
  Encoder2.msg
  EncoderStamped.msg
  Imu.msg
  ImuStamped.msg
  ImuRaw.msg
  ImuRawStamped.msg
  Led.msg
  PidParameters.msg
  Proximity.msg
  Vector3_32.msg
  Velocity.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)
