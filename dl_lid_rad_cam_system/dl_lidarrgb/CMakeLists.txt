cmake_minimum_required(VERSION 3.0.2)
project(dl_lidarrgb)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  pcl_conversions
  pcl_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dl_lidarrgb
#  CATKIN_DEPENDS cv_bridge image_transport pcl_conversions pcl_msgs pcl_ros roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


 add_executable(${PROJECT_NAME} src/dl_lidarrgb.cpp)


 target_link_libraries(${PROJECT_NAME}	
   ${catkin_LIBRARIES}
 )
 
 FIND_LIBRARY(mylib_LIBRARIES libL3Cam.a include/)
 target_link_libraries(${PROJECT_NAME} ${mylib_LIBRARIES})


