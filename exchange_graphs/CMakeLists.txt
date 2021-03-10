cmake_minimum_required(VERSION 3.0.2)
project(exchange_graphs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  ptp_comms
  std_msgs
  rosmsg_compressor
  graph_msgs
)

add_message_files(
  FILES
  GeometryGraphStamped.msg
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES exchange_graphs
  CATKIN_DEPENDS roscpp std_msgs rosmsg_compressor ptp_comms graph_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  include/${PROJECT_NAME}
  include/${PROJECT_NAME}/messages
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/exchange_graphs_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
