#include <ros/ros.h>



//#include <odom_rgbd_sync.h>
#include <ros/ros.h>
#include "sensor_msgs/PointCloud.h"

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/impl/ransac.hpp>
 
//Header file for writing PCD file 
#include<pcl/io/pcd_io.h> 

//message filter libraries
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "sensor_msgs/LaserScan.h"



// Include opencv2
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

 // Include CvBridge, Image Transport, Image msg
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
 
 /////////////////
#include <vector>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <tuple>
#include <string>
#include <std_msgs/String.h>
#include <iostream>
#include <fstream>
#include <tf/transform_broadcaster.h>
#include <dirent.h>
#include <camera_info_manager/camera_info_manager.h>

#include <omp.h>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/Marker.h>
 
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include <geometry_msgs/Vector3Stamped.h>

using namespace sensor_msgs;
using namespace message_filters;
/*Standard library*/
using namespace std;


ros::Publisher PC2_pub_;
ros::Publisher PC2m_pub_;
ros::Publisher lidarcenter_pub_;

vector<float> posx_; 
vector<float> posy_;
vector<float> posz_;

vector<float> posmx_; 
vector<float> posmy_;
vector<float> posmz_;

boost::numeric::ublas::vector<float> pointp_(3);

visualization_msgs::Marker lidarc_marker;
int k;
int cnnt=1;
