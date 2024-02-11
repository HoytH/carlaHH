cmake_minimum_required(VERSION 3.10)
project(HH)
target_link_directories(HH PUBLIC ${CMAKE_SOURCE_DIR}/)
add_library(HH parkingLotGraph.cpp)
