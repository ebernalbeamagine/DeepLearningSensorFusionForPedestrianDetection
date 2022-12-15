#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <iostream>
#include <locale>
#include <string>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>

#include <iostream>
#include <sys/time.h>
#include <ctime>

// Include opencv2
 #include <opencv2/imgproc/imgproc.hpp>
 #include <opencv2/highgui/highgui.hpp>

 // Include CvBridge, Image Transport, Image msg
 #include <image_transport/image_transport.h>
 #include <cv_bridge/cv_bridge.h>
 #include <sensor_msgs/image_encodings.h>
 
 #include "dl_lidar_ymdhmsz/lidarprocess.h"
 
//Declare the subscribers
ros::Subscriber lid_sub_;


//Declare the publishers
ros::Publisher lid_pub_;

ros::ServiceServer lidarservice;


bool flag_lidar   = false;
