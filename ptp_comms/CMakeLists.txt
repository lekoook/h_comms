cmake_minimum_required(VERSION 3.0.2)
project(ptp_comms)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  subt_communication_broker
)

add_message_files(
  FILES
  RxData.msg
)

add_service_files(
  FILES
  TxData.srv
  RegisterPort.srv
  Neighbors.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp message_runtime message_generation std_msgs
  DEPENDS system_lib
)

include_directories(
  include
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/serialisers.cpp)
add_executable(${PROJECT_NAME}_node src/ptp_comms_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} subt_communication_broker ${PROJECT_NAME})