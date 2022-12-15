cmake_minimum_required(VERSION 3.0.2)
project(dl_image_ymdhmsz)

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
    imgprocess.srv
 )


 generate_messages(
   DEPENDENCIES
   std_msgs
 )



catkin_package(
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport message_filters message_generation message_runtime nav_msgs pcl_conversions pcl_msgs pcl_ros roscpp rospy sensor_msgs std_msgs tf message_runtime
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)



add_executable(dl_image_ymdhmsz src/dl_image_ymdhmsz.cpp)
add_executable(dl_image_start_pause_client src/dl_image_start_pause_client.cpp)


add_dependencies(dl_image_ymdhmsz image_ymdhmsz_generate_messages_cpp)
add_dependencies(dl_image_start_pause_client dl_image_ymdhmsz_generate_messages_cpp)


target_link_libraries(dl_image_ymdhmsz ${catkin_LIBRARIES})
target_link_libraries(dl_image_start_pause_client ${catkin_LIBRARIES})


