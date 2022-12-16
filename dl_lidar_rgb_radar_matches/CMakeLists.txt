cmake_minimum_required(VERSION 3.0.2)
project(dl_lidar_rgb_radar_matches)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  camera_info_manager
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lidar_rgb_radar_matches
  CATKIN_DEPENDS actionlib actionlib_msgs camera_info_manager cv_bridge image_transport pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(dl_lidar_rgb_radar_matches src/dl_lidar_rgb_radar_matches.cpp)


target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
 )


