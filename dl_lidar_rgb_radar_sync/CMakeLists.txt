project(dl_lidar_rgb_radar_sync)


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
    process.srv
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


 add_executable(${PROJECT_NAME} src/dl_lidar_rgb_radar_sync.cpp)


 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
 )


