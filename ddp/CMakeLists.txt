cmake_minimum_required(VERSION 3.0.2)
project(ddp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  ptp_comms
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ddp
  CATKIN_DEPENDS ptp_comms roscpp
)

include_directories(
  include
  include/${PROJECT_NAME}
  include/${PROJECT_NAME}/requestors
  include/${PROJECT_NAME}/responders
  lib/serialisers/
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node 
  src/ddp_node.cpp 
  src/requestors/ReqMachine.cpp 
  src/requestors/ReqStates.cpp 
  src/responders/RespMachine.cpp 
  src/responders/RespStates.cpp
)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})