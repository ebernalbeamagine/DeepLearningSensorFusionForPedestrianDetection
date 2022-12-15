cmake_minimum_required(VERSION 3.0.2)
project(dl_lidar_ymdhmsz)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  message_filters
  message_generation
  message_runtime
  nav_msgs
  pcl_conversions
  pcl_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)


 find_package(Boost REQUIRED COMPONENTS system)



add_service_files(
   FILES
    lidarprocess.srv
#   Service1.srv
#   Service2.srv
 )




 generate_messages(
   DEPENDENCIES
    std_msgs
#   geometry_msgs#   nav_msgs#   pcl_msgs#   sensor_msgs#   std_msgs
 )




catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lidar_rgb_radar_sync
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport message_filters message_generation message_runtime nav_msgs pcl_conversions pcl_msgs pcl_ros roscpp rospy sensor_msgs std_msgs tf message_runtime
#  DEPENDS system_lib
)



include_directories(
   include
  ${catkin_INCLUDE_DIRS}
   ${Boost_INCLUDE_DIRS}
)




add_executable(dl_lidar_ymdhmsz src/dl_lidar_ymdhmsz.cpp)
add_executable(dl_lidar_start_pause_client src/dl_lidar_start_pause_client.cpp)



add_dependencies(dl_lidar_ymdhmsz dl_lidar_ymdhmsz_generate_messages_cpp)
add_dependencies(dl_lidar_start_pause_client dl_lidar_ymdhmsz_generate_messages_cpp)

 
target_link_libraries(dl_lidar_ymdhmsz ${catkin_LIBRARIES})
target_link_libraries(dl_lidar_start_pause_client ${catkin_LIBRARIES})


