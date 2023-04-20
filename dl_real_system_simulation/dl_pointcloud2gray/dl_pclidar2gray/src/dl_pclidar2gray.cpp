#include <dl_pclidar2gray.h>



void LidarCallback(const  sensor_msgs::PointCloud2ConstPtr& lidar_msg_){
   depth_ = cv::Mat::zeros(cv::Size(W, H), CV_16UC1);
   int posx_=-1;
   int posy_=-1;
   float d = 0;
   uint16_t d_int = 0 ;
   int kk=0; 
    
   //std::cout<<"Callback is here " << "\n";

   sensor_msgs::PointCloud pc_;
   

   sensor_msgs::convertPointCloud2ToPointCloud(*lidar_msg_,pc_);


   for (auto& point : pc_.points){
   
   
      lidar_point(0) =  -(float)(1000.0*(point.y));
      lidar_point(1) =  -(float)(1000.0*(point.z));
      lidar_point(2) =   (float)(1000.0*(point.x));
      lidar_point(3) = 1.0;
   
      camera_point(0) = ((lidar_point(0)) ? lidar_point(0) * fusion_matrix(0,0) : 0) + ((lidar_point(1)) ? lidar_point(1) * fusion_matrix(1,0) : 0) + ((lidar_point(2)) ? lidar_point(2) * fusion_matrix(2,0) : 0) + fusion_matrix(3,0); //lidar_point(3) * fusion_matrix(3,0);
      camera_point(1) = ((lidar_point(0)) ? lidar_point(0) * fusion_matrix(0,1) : 0) + ((lidar_point(1)) ? lidar_point(1) * fusion_matrix(1,1) : 0) + ((lidar_point(2)) ? lidar_point(2) * fusion_matrix(2,1) : 0) + fusion_matrix(3,1); //lidar_point(3) * fusion_matrix(3,1);
      camera_point(2) = ((lidar_point(0)) ? lidar_point(0) * fusion_matrix(0,2) : 0) + ((lidar_point(1)) ? lidar_point(1) * fusion_matrix(1,2) : 0) + ((lidar_point(2)) ? lidar_point(2) * fusion_matrix(2,2) : 0) + fusion_matrix(3,2); //lidar_point(3) * fusion_matrix(3,2);
    
      posx_ = camera_point(0)/camera_point(2);
      posy_ = camera_point(1)/camera_point(2);
   
    //std::cout << " posx	 " << posx_ << " posy " << posy_<<"\n";;
     
      d = sqrt(pow(lidar_point(0), 2.0) + pow(lidar_point(1), 2.0) + pow(lidar_point(2), 2.0))/1;
      d_int = (uint16_t)d;
      //getchar();
      depth_.at<uint16_t>(posy_,posx_) = d_int;
      kk++;
    
   
}

 ////////////////////////////////////////////////////////////////////////
 ///////---convert depth to a ROS image and send it for the SegNet---////
 //////////////////////////////////////////////////////////////////////// 
   depth_image          = cv::Mat::zeros(H,W,CV_16UC1);
   depth_image_double   = cv::Mat::zeros(H,W,CV_16UC1);
   depth_image_gray     = cv::Mat::zeros(H,W,CV_16UC1);
   depth_image_gray8    = cv::Mat::zeros(H,W,CV_8UC1);
 
    
 for (int i = 0; i < H; i++) {
    uint16_t* depth_ptr = depth_.ptr<uint16_t>(i);
    uint16_t* depth_image_ptr = depth_image.ptr<uint16_t>(i);
    for (int j = 0; j < W; j++) {
        if (*depth_ptr != 0) {
            *depth_image_ptr = *depth_ptr;
        }
        depth_ptr++;
        depth_image_ptr++;
    }
}
    
   //Initializing max element as INT_MIN
    uint16_t maxElement = 0;   
    const uint16_t* ptr = depth_image.ptr<uint16_t>(0);
    const uint16_t* end = ptr + H * W;

    for (; ptr < end; ptr++) {
       if (*ptr > maxElement) {
        maxElement = *ptr;
       }
     }
   
   for(int i = 0; i < H; i++){
       uint16_t* depth_ptr = depth_.ptr<uint16_t>(i);
       uint16_t* depth_image_ptr = depth_image_double.ptr<uint16_t>(i);
       for(int j = 0; j < W; j++){
          if(*depth_ptr != 0){
             *depth_image_ptr = *depth_ptr;
          }
          depth_ptr++;
          depth_image_ptr++;
       }
   }
  
   depth_image_double.convertTo(depth_image_double, CV_64F);
 
     for (int i = 0; i < H; i++) {
        int16_t* depth_row = depth_image_gray.ptr<int16_t>(i);
        const double* depth_double_row = depth_image_double.ptr<double>(i);
        for (int j = 0; j < W; j++) {
            depth_row[j] = int16_t((depth_double_row[j] / float(maxElement)) * 65536);
       }
   }
    
   //Create image color
   cv::Mat image_rgb(H, W, 3);   
  
   // convert the gray image to rgb
   if (depth_image_gray.type()==CV_16UC1){ 
      cvtColor(depth_image_gray, image_rgb, CV_GRAY2BGR);
   }
   
   
   
   string ty =  openCVType2str(image_rgb.type() );
  
      
      
   cv_bridge::CvImage img_bridge;
   sensor_msgs::Image img_msg; // >> message to be sent

   std_msgs::Header header; // empty header
   header.stamp = ros::Time::now(); // time
   header.frame_id  =   "lidar";
   
   img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO16, depth_image_gray);
   img_bridge.toImageMsg(img_msg); 
   
   
   img_pub_.publish(img_msg); 
    
   lidar_cnt++;
}

string openCVType2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}


         


int main(int argc, char **argv){

ros::init(argc, argv, "dl_pclidar2gray");

  
  ros::NodeHandle nh_;
  
   fusion_matrix.resize(4,3);
   lidar_homogeneous.resize(4,3);
   camera_intrinsics.resize(3,3);
   
   lidar_homogeneous(0,0) =  0.999978;   
   lidar_homogeneous(0,1) = -0.000689;
   lidar_homogeneous(0,2) =  -0.006672;
   lidar_homogeneous(1,0) = 0.000203;   
   lidar_homogeneous(1,1) = 0.997365;
   lidar_homogeneous(1,2) = -0.072545;
   lidar_homogeneous(2,0) = 0.006704;
   lidar_homogeneous(2,1) =  0.072542;
   lidar_homogeneous(2,2) = 0.997343;
   lidar_homogeneous(3,0) = 1.332946; 
   lidar_homogeneous(3,1) =  -4.700044;
   lidar_homogeneous(3,2) =  -102.268624;
   
   //----- CAMERA INTRINSIC MATRIX [fx 0 0; s fy 0; cx cy 1] -----
   camera_intrinsics(0,0) = 1329.715963;
   camera_intrinsics(0,1) = 0.000000;
   camera_intrinsics(0,2) = 0.000000;
   camera_intrinsics(1,0) = 0.000000;
   camera_intrinsics(1,1) = 1330.014514;
   camera_intrinsics(1,2) = 0.000000;
   camera_intrinsics(2,0) = 967.180398;
   camera_intrinsics(2,1) = 561.342722;
   camera_intrinsics(2,2) = 1.000000;
   
   
   fusion_matrix= boost::numeric::ublas::prod(lidar_homogeneous, camera_intrinsics);
  
  
 
   lidar_     = nh_.subscribe("/lidar_simulation", 10, LidarCallback);
   img_pub_   = nh_.advertise<sensor_msgs::Image>("/lidar_interpolated16", 2);
  
  ros::Rate loop_rate(6);
  
   int count = 0;

while (ros::ok()){
   
    ros::spinOnce();
    loop_rate.sleep();
   
}
  
   pc_pub_.shutdown();
   lidar_.shutdown();
   img_pub_.shutdown();
   return 0;
  
}
