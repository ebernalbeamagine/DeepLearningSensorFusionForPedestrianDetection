cmake_minimum_required(VERSION 3.0.2)
project(dl_libl3cam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  message_generation
  message_runtime
)

## System dependencies are found with CMake's conventions
 find_package(Boost REQUIRED COMPONENTS system)




## Generate services in the 'srv' folder
 add_service_files(
   FILES
   process.srv
#   Service1.srv
#   Service2.srv
 )



## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   sensor_msgs
   std_msgs
 )


## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES dl_libl3cam
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs tf message_runtime
#  DEPENDS system_lib
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME} src/dl_libl3cam.cpp)



## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
 )
 
 FIND_LIBRARY(mylib_LIBRARIES libL3Cam.a include/)
 target_link_libraries(${PROJECT_NAME} ${mylib_LIBRARIES})


