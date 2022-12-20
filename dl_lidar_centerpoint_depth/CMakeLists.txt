cmake_minimum_required(VERSION 3.0.2)
project(dl_lidar_centerpoint_depth)


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
  visualization_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lidar_centerpoint_depth
#  CATKIN_DEPENDS cv_bridge image_transport pcl_conversions pcl_msgs pcl_ros roscpp rospy sensor_msgs std_msgs tf visualization_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME} src/dl_lidar_centerpoint_depth.cpp)


 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
 )


