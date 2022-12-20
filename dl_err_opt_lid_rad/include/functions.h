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

#include <iostream>     // std::cout
#include <fstream>      // std::ifstream

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/impl/ransac.hpp>
#include <pcl/common/common.h>
#include <pcl/common/common_headers.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/passthrough.h>

#include <tf/transform_broadcaster.h>
#include "sensor_msgs/PointCloud.h"

#include <sensor_msgs/image_encodings.h>
#include <dirent.h>
#include <camera_info_manager/camera_info_manager.h>


#include<algorithm>



#include <geometry_msgs/Vector3Stamped.h>
#include <opencv4/opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>

#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include <visualization_msgs/Marker.h>
#include <tuple>



using namespace std; 




vector<std::string>  GetAllImagesFromFolder(std::string folder_pathi_);
vector<std::string> GetAllLidarPointCloudFromFolder(std::string folder_pathl_);
vector<std::string> GetAllRadarPointCloudFromFolder(std::string folder_pathr_);
string type2str(int type);
cv::Mat ImagePublisher(vector<std::string> file_name_listimg_,ros::Publisher img_pub_);
std::tuple<float, float> ImageCenterBoardPosition(cv::Mat image_g);
std::tuple<float, float, float> DirectionalVector(vector<std::string> file_name_listl_, ros::Time time, int cntl, float x, float y, bool gray16, bool gray8);
inline void invertMatrix(const boost::numeric::ublas::matrix<float> &input, boost::numeric::ublas::matrix<float> &output);
inline boost::numeric::ublas::vector<float> undistortPoint(float x, float y);
boost::numeric::ublas::vector<float> directional_vector(float dir_x, float dir_y);



















