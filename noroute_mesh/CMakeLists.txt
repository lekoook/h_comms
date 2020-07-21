cmake_minimum_required(VERSION 3.0.2)
project(aodv)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  message_generation
  subt_communication_broker
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  send_map.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
)

catkin_package(
   CATKIN_DEPENDS message_generation message_runtime std_msgs nav_msgs
)

include_directories(
  include
  eth
  lib
  node
  utilities
  ${catkin_INCLUDE_DIRS}
)

add_library(eth eth/Eth.cpp eth/Eth.hpp)
add_library(crc lib/crc32.c)
add_library(node node/Node.cpp node/Node.hpp)
add_library(serialisers utilities/serialisers.cpp utilities/serialisers.hpp)

add_executable(aodv_node src/main.cpp)

target_link_libraries(aodv_node
  eth
  crc
  node
  serialisers
  ${catkin_LIBRARIES}
)

#add_executable(sim_node main.cpp)

#target_link_libraries(sim_node
#  eth
#  crc
#  node
# rerr
#  rrep
#  rrep_ack
#  rreq
#  table
#  serialisers
#  ${catkin_LIBRARIES}
#)
