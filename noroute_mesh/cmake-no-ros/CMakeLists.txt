cmake_minimum_required(VERSION 3.0.2)
project(aodv)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -ggdb")

include_directories(
  ../include
  ../eth
  ../lib
  ../node
  ../utilities
)

add_library(eth ../eth/Eth.cpp ../eth/Eth.hpp)
add_library(crc ../lib/crc32.c)
add_library(node ../node/Node.cpp ../node/Node.hpp)
add_library(serialisers ../utilities/serialisers.cpp ../utilities/serialisers.hpp)
add_library(sim ../utilities/Sim.cpp ../utilities/Sim.hpp)
add_library(parson ../lib/parson.c ../lib/parson.h)

add_executable(aodv ../main.cpp)

target_link_libraries(eth
  serialisers
)

target_link_libraries(node
  eth
)

target_link_libraries(sim
  node
  parson
)

target_link_libraries(aodv
  sim
)
