cmake_minimum_required(VERSION 3.0.2)
project(dl_sensors2_8_16_gray)


find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  camera_info_manager
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
#  LIBRARIES sensors2_8_16_gray
  CATKIN_DEPENDS actionlib actionlib_msgs camera_info_manager cv_bridge image_transport pcl_conversions pcl_msgs pcl_ros roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME} src/dl_sensors2_8_16_gray.cpp)


 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
 )


