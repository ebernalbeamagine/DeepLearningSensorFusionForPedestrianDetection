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
int W=1920;
int H=1080;

int size;


vector<std::string>   file_name_listr_;

cv::Mat depthr_ = cv::Mat::zeros(cv::Size(W,H), CV_16UC1);

boost::numeric::ublas::vector<float> radar_point(4);

ros::Publisher ra_pub_;

boost::numeric::ublas::matrix<float> radar_transformation_;
