#include <sstream>
#include <vector>
#include <opencv4/opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>

#include <sensor_msgs/PointCloud2.h>
#include <tuple>

#include <sensor_msgs/point_cloud_conversion.h>
#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <fstream>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/PointCloud.h"

#include <sensor_msgs/image_encodings.h>
#include <dirent.h>
#include <camera_info_manager/camera_info_manager.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/impl/ransac.hpp>

#include <opencv4/opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>

#include <pcl/common/common.h>
#include <pcl/common/common_headers.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include <Eigen/Geometry>

using namespace std; 
using std::vector;
using namespace cv;

vector<std::string>   file_name_listi_;
vector<std::string>   file_name_listl_;
vector<std::string>   file_name_listr_;

int W=1920;
int H=1080;

//Define cv matrices 
cv::Mat depthr_ = cv::Mat::zeros(cv::Size(W,H), CV_16UC1);
cv::Mat depthr_image; 
cv::Mat depthr_image_double;
cv::Mat depthr_image_gray; 
cv::Mat depthr_image_gray8;

cv::Mat depth_ = cv::Mat::zeros(cv::Size(W,H), CV_16UC1);
cv::Mat depth_interpolate  = cv::Mat::zeros(W,H,CV_8UC1);
cv::Mat depth_image; 
cv::Mat depth_image_double;
cv::Mat depth_image_gray; 
cv::Mat image;
cv::Mat image_undistorted;
cv::Mat camera_matrix_;
cv::Mat rgb_distortion_coef_;
cv::Mat depth_image_gray8;

//Define boost matrices

boost::numeric::ublas::matrix<float> radar_transformation_;



////new matrices

boost::numeric::ublas::vector<float> lidar_point(4);
boost::numeric::ublas::vector<float> radar_point(4);
boost::numeric::ublas::vector<float> camera_point(3);
boost::numeric::ublas::vector<float> camerar_point(3);
boost::numeric::ublas::matrix<float> camera_intrinsics;
boost::numeric::ublas::matrix<float> lidar_homogeneous;
boost::numeric::ublas::matrix<float> fusion_matrix;

//parameters

int lidar_cnt = 0;
int radar_cnt = 0;
int img_cnt = 0;
int leading = 3;
int size;

int counter=0;

Eigen::Matrix4d m;	


bool gray16;
bool gray8;
bool gray16_flag = true;
bool gray8_flag = true;

struct POINT_COORDINATE{
    float x;
    float y;
    float z;
  };
vector<POINT_COORDINATE>  pos_coordinate_xyz_; 

pcl::PointCloud<pcl::PointXYZ>::Ptr global_point_cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr ra_global_point_cloud (new pcl::PointCloud<pcl::PointXYZ>);


ros::Publisher chatter_pub;
ros::Publisher li_pub_; 
ros::Publisher ra_pub_; 


vector<int> rad_img_x;
vector<int> rad_img_y;
