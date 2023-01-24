cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(l3cam_simulator)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_definitions(${OpenCV_DEFINITIONS})

find_package(Threads REQUIRED)

add_executable (l3cam_simulator main.cpp udpSender.cpp)

target_link_libraries( l3cam_simulator ${OpenCV_LIBS} "${CMAKE_THREAD_LIBS_INIT}" )

