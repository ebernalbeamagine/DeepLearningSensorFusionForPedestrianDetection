//#include <odom_rgbd_sync.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "sensor_msgs/PointCloud.h"

// PCL specific includes
 #include <pcl_conversions/pcl_conversions.h>
 #include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <sensor_msgs/point_cloud_conversion.h>
 
//Header file for writing PCD file 
#include<pcl/io/pcd_io.h> 

//message filter libraries
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "sensor_msgs/LaserScan.h"
#include "lidar_rgb_radar_sync/process.h"

// Include opencv2
 #include <opencv2/imgproc/imgproc.hpp>
 #include <opencv2/highgui/highgui.hpp>

 // Include CvBridge, Image Transport, Image msg
 #include <image_transport/image_transport.h>
 #include <cv_bridge/cv_bridge.h>
 #include <sensor_msgs/image_encodings.h>
 
 

using namespace sensor_msgs;
using namespace message_filters;
/*Standard library*/
using namespace std;

ros::Publisher pc2_pub_;
ros::Publisher img_pub_;
ros::Publisher radar_pub_;
ros::ServiceServer service;


bool flag_process = false;
bool flag_lidar   = false;
bool flag_radar   = false;
bool flag_rgb     = false;

int cnt       = 0;
int lidar_cnt = 0;
int radar_cnt = 0;
int leading   = 3;
