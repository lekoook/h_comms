cmake_minimum_required(VERSION 3.0.2)
project(aodv)

include_directories(
  ../include
  ../eth
  ../lib
  ../node
  ../rerr
  ../rrep
  ../rrep_ack
  ../rreq
  ../table
  ../utilities
)

add_library(eth ../eth/Eth.cpp ../eth/Eth.hpp)
add_library(crc ../lib/crc32.c)
add_library(node ../node/Node.cpp ../node/Node.hpp)
add_library(rerr ../rerr/Rerr.cpp ../rerr/Rerr.hpp)
add_library(rrep ../rrep/Rrep.cpp ../rrep/Rrep.hpp)
add_library(rrep_ack ../rrep_ack/RrepAck.cpp ../rrep_ack/RrepAck.hpp)
add_library(rreq ../rreq/Rreq.cpp ../rreq/Rreq.hpp)
add_library(sim ../utilities/Sim.cpp ../utilities/Sim.hpp)
add_library(table ../table/Table.cpp ../table/Table.hpp)
add_library(serialisers ../utilities/serialisers.cpp ../utilities/serialisers.hpp)

add_executable(aodv ../main.cpp)

target_link_libraries(aodv
  eth
  crc
  node
  rerr
  rrep
  rrep_ack
  rreq
  serialisers
  sim
  table
)
