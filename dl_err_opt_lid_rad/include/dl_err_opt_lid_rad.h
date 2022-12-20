Eigen::Matrix4d m;

struct RAD_COORDINATE{
    float x;
    float y;
    float z;
  };


vector<RAD_COORDINATE>   rad_xyz_;


using namespace std; 
using std::vector;
using namespace cv;

vector<float> radar_posx_; 
vector<float> radar_posy_;
vector<float> radar_posz_;


vector<float> lidar_posx_; 
vector<float> lidar_posy_;
vector<float> lidar_posz_;

float yo_min;
float yo_max;
float y_min;
float y_max;


int    kl        = 0;
int    kr        = 0;


bool gray16;
bool gray8;

ros::Publisher li_pub_; 
ros::Publisher ra_pub_; 
ros::Publisher im_pub_; 
ros::Publisher PC2_pub_;
ros::Publisher img_pub_;
ros::Publisher dirv_pub_;
ros::Publisher chatter_pub;
ros::Publisher radarcenter_pub_;
visualization_msgs::Marker radarc_marker;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_li (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ra (new pcl::PointCloud<pcl::PointXYZ>);

inline boost::numeric::ublas::vector<float> undistortPoint(float x, float y);
boost::numeric::ublas::vector<float> directional_vector(float dir_x, float dir_y);
boost::numeric::ublas::vector<float> pointp_(3);
boost::numeric::ublas::vector<float> radar_point(4);
boost::numeric::ublas::matrix<float> radar_transformation_;


inline void invertMatrix(const boost::numeric::ublas::matrix<float> &input, boost::numeric::ublas::matrix<float> &output);



