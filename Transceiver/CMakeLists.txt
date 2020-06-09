cmake_minimum_required(VERSION 3.10)
project(Transceiver)

add_library(CTransceiver CTransceiver.cpp CTransceiver.hpp)
add_library(Transceiver Transceiver.hpp)
set_target_properties(Transceiver PROPERTIES LINKER_LANGUAGE CXX)
add_executable(trans main.cpp)

target_link_libraries(trans PRIVATE Transceiver CTransceiver)