


#include <dl_pcradar2gray.h>


void RadarCallback(const  sensor_msgs::PointCloud2ConstPtr& radar_msg_){
    depthr_ = cv::Mat::zeros(cv::Size(W, H), CV_16UC1);
   
   int kk=0; 
   int r_posx_=-1;
   int r_posy_=-1;
   float dr = 0;
   uint16_t dr_int = 0 ;
    
   //std::cout<<"Callback is here " << "\n";

   sensor_msgs::PointCloud pc_;
 
   sensor_msgs::convertPointCloud2ToPointCloud(*radar_msg_,pc_);
 
    for (auto& point : pc_.points){
 
      radar_point(0) =  -(float)(1000.0*(point.y));
      radar_point(1) =  -(float)(1000.0*(point.z));
      radar_point(2) =   (float)(1000.0*(point.x));
      radar_point(3) = 1.0;
 
      
      camerar_point(0) = ((radar_point(0)) ? radar_point(0) * fusion_matrix(0,0) : 0) + ((radar_point(1)) ? radar_point(1) * fusion_matrix(1,0) : 0) + ((radar_point(2)) ? radar_point(2) * fusion_matrix(2,0) : 0) + fusion_matrix(3,0); //lidar_point(3) * fusion_matrix(3,0);
      camerar_point(1) = ((radar_point(0)) ? radar_point(0) * fusion_matrix(0,1) : 0) + ((radar_point(1)) ? radar_point(1) * fusion_matrix(1,1) : 0) + ((radar_point(2)) ? radar_point(2) * fusion_matrix(2,1) : 0) + fusion_matrix(3,1); //lidar_point(3) * fusion_matrix(3,1);
      camerar_point(2) = ((radar_point(0)) ? radar_point(0) * fusion_matrix(0,2) : 0) + ((radar_point(1)) ? radar_point(1) * fusion_matrix(1,2) : 0) + ((radar_point(2)) ? radar_point(2) * fusion_matrix(2,2) : 0) + fusion_matrix(3,2); //lidar_point(3) * fusion_matrix(3,2);
    
   r_posx_ = camerar_point(0)/camerar_point(2);
   r_posy_ = camerar_point(1)/camerar_point(2);
  
   dr = sqrt(pow(radar_point(0), 2.0) + pow(radar_point(1), 2.0) + pow(radar_point(2), 2.0))/1;
   if(dr>35000)
       continue;

    
    dr_int = (uint16_t)dr;

     
    if((r_posx_ > W) || (r_posx_< 0))
         continue;
     if((r_posy_ > H) || (r_posy_ < 0))
         continue;
 
   
    depthr_.at<uint16_t>(r_posy_,r_posx_) = dr_int;

    
    for(int i = 0; i < H; i++){
         int j = r_posx_;
  
            depthr_.at<uint16_t>(i,j)= dr_int;
          
    } 
      
   }
   
  ////////////////////////////////////////////////////////////////////////
 ///////---convert depth to a ROS image and send it for the SegNet---////
 //////////////////////////////////////////////////////////////////////// 
   depth_image          = cv::Mat::zeros(H,W,CV_16UC1);
   depth_image_double   = cv::Mat::zeros(H,W,CV_16UC1);
   depth_image_gray     = cv::Mat::zeros(H,W,CV_16UC1);
   depth_image_gray8    = cv::Mat::zeros(H,W,CV_8UC1);

 
    for (int i = 0; i < H; i++) {
    uint16_t* depth_ptr = depthr_.ptr<uint16_t>(i);
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
       uint16_t* depth_ptr = depthr_.ptr<uint16_t>(i);
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
   
 
   cv_bridge::CvImage img_bridge;
   sensor_msgs::Image img_msg; // >> message to be sent

   std_msgs::Header header; // empty header
   ///header.seq = counter; // user defined counter
   header.stamp = ros::Time::now(); // time
   header.frame_id  =   "lidar";
   //img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO16, depth16_interpolated);
   img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO16, depth_image_gray);
   
   img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
   
   
   img_pub_.publish(img_msg); 
    
   radar_cnt++;
   
}


int main(int argc, char **argv){

   ros::init(argc, argv, "dl_pcradar2gray");

  
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
   
   
   
   ros::Rate loop_rate(10);
   
   radar_ = nh_.subscribe("/radar_simulation", 10, RadarCallback);
   img_pub_   = nh_.advertise<sensor_msgs::Image>("/radar_interpolated16", 2);  
  
  
   int count = 0;

while (ros::ok()){
 

    ros::spinOnce();
    loop_rate.sleep();
   
  }
  
   radar_.shutdown();
   img_pub_.shutdown();
   return 0;
  
}
