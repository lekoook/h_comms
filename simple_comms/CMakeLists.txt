cmake_minimum_required(VERSION 3.0.2)
project(simple_comms)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  subt_communication_broker
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(comms_time src/comms_time.cpp)

add_executable(simple_comms src/simple_comms.cpp)
target_link_libraries(simple_comms comms_time ${catkin_LIBRARIES})
