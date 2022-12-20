#include<functions.h>
using namespace std; 
using std::vector;
using namespace cv;

vector<std::string>   file_name_listi_;
vector<std::string>   file_name_listl_;
vector<std::string>   file_name_listr_;

int cnt=0;

boost::numeric::ublas::matrix<float> camera_intrinsics_inv_;
boost::numeric::ublas::vector<float> lidar_point(4);
boost::numeric::ublas::vector<float> camera_point(3);
boost::numeric::ublas::matrix<float> fusion_matrix;
boost::numeric::ublas::matrix<float> camera_intrinsics;
boost::numeric::ublas::matrix<float> lidar_homogeneous;
boost::numeric::ublas::matrix<float> camera_to_lidar_rotation_; 
boost::numeric::ublas::matrix<float> camera_to_lidar_transformation_; 
boost::numeric::ublas::matrix<float> lidar_to_camera_transformation_;

cv::Mat camera_matrix_;
cv::Mat rgb_distortion_coef_;
cv::Mat depth_image; 
cv::Mat depth_image_double;
cv::Mat depth_image_gray; 
cv::Mat depth_image_gray8;


bool flag_init   = true;
bool gray16_flag = true;
bool gray8_flag  = true;


int lidar_cnt = 0;
int cnnt=1;

float clx;
float cly;
float clz;

///////////////////////////////////////////
///---Get the path of all RGB images    ///
///////////////////////////////////////////
vector<std::string> GetAllImagesFromFolder(std::string folder_pathi_){
   std::string full_pathi_;
   file_name_listi_.clear();
   std::cout<<"Reading images at: " << folder_pathi_ << std::endl;
   DIR* FDi_;
   struct dirent* current_filei_;
   std::string current_file_namei_;
   if (NULL == (FDi_ = opendir(folder_pathi_.c_str()))){
      perror("opendir() error");
      std::cout<<"Failed to open the image directory"<<std::endl;    
   }
   else{
      while((current_filei_ = readdir(FDi_))){
         if(!strcmp (current_filei_->d_name, "."))
            continue;
               if(!strcmp (current_filei_->d_name, ".."))    
                  continue;
            
            //! Get extension of the file
            current_file_namei_ = current_filei_->d_name;
            size_t pos = current_file_namei_.find('.');

            if(pos != std::string::npos){
               std::string ext = current_file_namei_.substr((pos+1));
               if(ext.compare("png") == 0 ){
                  full_pathi_ = folder_pathi_ + current_file_namei_;
                  file_name_listi_.push_back(full_pathi_);
               }
            }
      }
      if(closedir(FDi_)==0)
         std::cout<<"The dir FDi_ is closed successfully"<<std::endl;
   }
     
   //!Ordernar los ficheros 
   std::sort(file_name_listi_.begin(), file_name_listi_.end());
   std::cout<<"Total image files read: "<<file_name_listi_.size()<<std::endl; 
   return file_name_listi_;
}


/////////////////////////////////////////////
///---Get the path of all Lidar images    ///
/////////////////////////////////////////////
vector<std::string> GetAllLidarPointCloudFromFolder(std::string folder_pathl_){
   std::string full_pathl_;
   file_name_listl_.clear();
   std::cout<<"Reading images at: " << folder_pathl_ << std::endl;
   DIR* FDl_;
   struct dirent* current_filel_;
   std::string current_file_namel_;
   if (NULL == (FDl_ = opendir(folder_pathl_.c_str()))){
      perror("opendir() error");
      std::cout<<"Failed to open the laser directory"<<std::endl;    
   }
   else{
      while((current_filel_ = readdir(FDl_))) {
         if(!strcmp (current_filel_->d_name, "."))
            continue;
         if(!strcmp (current_filel_->d_name, ".."))    
            continue;
             
          //! Get extension of file
         current_file_namel_ = current_filel_->d_name;
         size_t pos = current_file_namel_.find('.');

         if(pos != std::string::npos){
            std::string ext = current_file_namel_.substr((pos+1));
            if(ext.compare("pcd") == 0 ){     
               full_pathl_ = folder_pathl_ + current_file_namel_;
               file_name_listl_.push_back(full_pathl_);  
            }
         }
      }
      if(closedir(FDl_)==0)
         std::cout<<"The dir FDl_ is closed successfully"<<std::endl;    
   }
   
   //!ordering the files
   std::sort(file_name_listl_.begin(), file_name_listl_.end());
   std::cout<<"Total laser files read: "<<file_name_listl_.size()<<std::endl; 
   
   return file_name_listl_;
}

/////////////////////////////////////////////
///---Get the path of all Radar images    ///
/////////////////////////////////////////////
vector<std::string> GetAllRadarPointCloudFromFolder(std::string folder_pathr_){
   std::string full_pathl_;
   file_name_listr_.clear();
   std::cout<<"Reading images at: " << folder_pathr_ << std::endl;
   DIR* FDl_;
   struct dirent* current_filel_;
   std::string current_file_namel_;
   if (NULL == (FDl_ = opendir(folder_pathr_.c_str()))){
      perror("opendir() error");
      std::cout<<"Failed to open the laser directory"<<std::endl;    
   }
   else{
      while((current_filel_ = readdir(FDl_))){
         if(!strcmp (current_filel_->d_name, "."))
            continue;
         if(!strcmp (current_filel_->d_name, ".."))    
            continue; 
         //! Get extension of file
         current_file_namel_ = current_filel_->d_name;
         size_t pos = current_file_namel_.find('.');

         if(pos != std::string::npos){
            std::string ext = current_file_namel_.substr((pos+1));

            if(ext.compare("pcd") == 0 ){     
               full_pathl_ = folder_pathr_ + current_file_namel_;
               file_name_listr_.push_back(full_pathl_);  
            }
         }
      }
      if(closedir(FDl_)==0)
         std::cout<<"The dir FDl_ is closed successfully"<<std::endl;    
   }
   
    //!Ordernar los ficheros 
    std::sort(file_name_listr_.begin(), file_name_listr_.end());
    std::cout<<"Total radar files read: "<<file_name_listr_.size()<<std::endl;
    
    return file_name_listr_;
}


//////////////////////////////////////
///---Get the type of the image    ///
//////////////////////////////////////
string type2str(int type) {
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



/////////////////////////////////////////////////////////////
//------Handle the image center position of the board------//
///////////////////////////////////////////////////////////// 
std::tuple<float, float> ImageCenterBoardPosition(cv::Mat image_g){

   float x;
   float y;
   
   if(cnnt<9){
      cv::medianBlur(image_g,image_g,15);
      vector<Vec3f> circles;
 
      HoughCircles(image_g, circles, CV_HOUGH_GRADIENT, 2,
                   image_g.rows/20,  // change this value to detect circles with different distances to each other
                   360, 60, 1, 35    // change the last two parameters
                                     // (min_radius & max_radius) to detect larger circles
                  );
    
      /* -------------- POINT coordinate --------------------------*/
      struct POINT_COORDINATE{
         float x;
         float y;
         float dd;
      };


      vector<POINT_COORDINATE>   pos_ref_xy_; 

      pos_ref_xy_.clear();
   
      for( size_t i = 0; i < circles.size(); i++ ){
         std::cout << "-----HERE2--- "<< "\n";  
         POINT_COORDINATE ref_xy_;
         Vec3i c       = circles[i];
         ref_xy_.x     = c[0];
         ref_xy_.y     = c[1];
         ref_xy_.dd    = sqrt(pow(c[0], 2.0) + pow(c[1], 2.0) );
         pos_ref_xy_.push_back(ref_xy_);
      }

      for(int i = 0; i < circles.size(); i++ ){
         for(int j = 0; j < circles.size(); j++ ){
            if( pos_ref_xy_[j].dd > pos_ref_xy_[i].dd ){
               float tmpd = pos_ref_xy_[i].dd; 
               float tmpx = pos_ref_xy_[i].x; 
               float tmpy = pos_ref_xy_[i].y; 
             
               pos_ref_xy_[i].dd = pos_ref_xy_[j].dd;
               pos_ref_xy_[i].x = pos_ref_xy_[j].x;
               pos_ref_xy_[i].y = pos_ref_xy_[j].y;
             
               pos_ref_xy_[j].dd = tmpd;
               pos_ref_xy_[j].x = tmpx;
               pos_ref_xy_[j].y = tmpy;
            }     
         }
      }

  
      float x1 = pos_ref_xy_[0].x;
      float y1 = pos_ref_xy_[0].y;
    
      float x2 = pos_ref_xy_[3].x;
      float y2 = pos_ref_xy_[3].y;
     
      float x3 = pos_ref_xy_[1].x;
      float y3 = pos_ref_xy_[1].y; 
   
      float x4 = pos_ref_xy_[2].x;
      float y4 = pos_ref_xy_[2].y;//c4[1];
    
      float A1 = y2-y1;
      float B1 = x1-x2;
      float C1 = y1*(x2-x1)-x1*(y2-y1);
   
      float A2 = y4-y3; 
      float B2 = x3-x4;
      float C2 = y3*(x4-x3)-x3*(y4-y3);
    
      float det = A1*B2 -A2*B1;
    
      x = (-C1*B2  + C2*B1)/det;
      y = (-A1*C2  + A2*C1)/det;
        
      for( size_t i = 0; i < circles.size(); i++ ){ 
         Vec3i c = circles[i];
         Point center = Point(c[0], c[1]);
         // circle center
         circle(image_g, center, 1, Scalar(0,100,100), 3, LINE_AA);
         // circle outline
         int radius = c[2];
         circle(image_g, center, radius, Scalar(255,0,255), 3, LINE_AA);
      }
            
      //Point origin = Point(667,683 );     
      //Point origin = Point(pos_ref_xy_[2].x,pos_ref_xy_[2].y);
      Point origin = Point(x,y);
      circle(image_g, origin, 1, Scalar(0,100,100), 3, LINE_AA);
      circle(image_g, origin, 28, Scalar(255,0,0), 3, LINE_AA);  
   }   
 
   std::cout << "-----COUNTTTT--- "<< cnnt<<"\n";
   if(cnnt==11){
       x=972;
       y=691;
   }
   
   cnnt++;
  
   return {x,y};

}    
/////////
//-END-//
/////////

/////////////////////////////////////////////////////////////////////////////////////////////////
//------This function Computes the directional vector, e.g. The vector                         //
//------from the center of the camera that points towards the center of                        //
//------the calibration board and also rotate the vector with respects to the lidar frame -----//
/////////////////////////////////////////////// 
std::tuple<float, float, float> DirectionalVector(vector<std::string> file_name_listl_, ros::Time time, int cntl, float x, float y, bool gray16, bool gray8){ 
   ////////////////////////////////////////////////   
   ////---All teh  parameters, and matrices---/////
   ////////////////////////////////////////////////
   camera_intrinsics.resize(3,3);
   lidar_homogeneous.resize(4,3);
   camera_to_lidar_rotation_.resize(3,3);
   camera_to_lidar_transformation_.resize(4,4);
   lidar_to_camera_transformation_.resize(4,4);
   camera_intrinsics_inv_.resize(3,3);
   
   camera_matrix_ = cv::Mat(3,3,CV_32F);
   rgb_distortion_coef_ = cv::Mat(1,5,CV_32F);
   
   rgb_distortion_coef_.row(0).col(0) =  0.091380;
   rgb_distortion_coef_.row(0).col(1) = -0.150430 ;
   rgb_distortion_coef_.row(0).col(2) =  -0.002450;
   rgb_distortion_coef_.row(0).col(3) =  -0.000005 ;
   rgb_distortion_coef_.row(0).col(4) =   0.000000;
   
   //----- CAMERA INTRINSIC MATRIX [fx 0 0; s fy 0; cx cy 1] ----- 
   camera_matrix_.row(0).col(0) = 1329.715963;
   camera_matrix_.row(0).col(1) = 0.000000;
   camera_matrix_.row(0).col(2) = 967.180398; 
   camera_matrix_.row(1).col(0) = 0.000000;
   camera_matrix_.row(1).col(1) = 1330.014514; 
   camera_matrix_.row(1).col(2) = 561.342722;
   camera_matrix_.row(2).col(0) = 0.000000;
   camera_matrix_.row(2).col(1) = 0.000000;
   camera_matrix_.row(2).col(2) = 1.000000;
   
   lidar_homogeneous(0,0) =    0.999978;   
   lidar_homogeneous(0,1) =   -0.000689;
   lidar_homogeneous(0,2) =   -0.006672;
   lidar_homogeneous(1,0) =    0.000203;   
   lidar_homogeneous(1,1) =    0.997365;
   lidar_homogeneous(1,2) =   -0.072545;
   lidar_homogeneous(2,0) =    0.006704;
   lidar_homogeneous(2,1) =    0.072542;
   lidar_homogeneous(2,2) =    0.997343;
   lidar_homogeneous(3,0) =    1.332946; 
   lidar_homogeneous(3,1) =   -4.700044;
   lidar_homogeneous(3,2) = -102.268624;
   
   
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
   
   invertMatrix(camera_intrinsics,camera_intrinsics_inv_ );
   
   
   //----- LIDAR TO CAMERA TRANSFORMATION (L to C) ----- 
   //----- p_cam = T*p_lid = [R | t]----- 
   lidar_to_camera_transformation_(0,0) =    0.999978;   
   lidar_to_camera_transformation_(0,1) =   -0.000689;   
   lidar_to_camera_transformation_(0,2) =   -0.006672;  
   lidar_to_camera_transformation_(0,3) =    0.00000;   
   lidar_to_camera_transformation_(1,0) =    0.000203;  
   lidar_to_camera_transformation_(1,1) =    0.997365;  
   lidar_to_camera_transformation_(1,2) =   -0.072545;   
   lidar_to_camera_transformation_(1,3) =    0.00000;  
   lidar_to_camera_transformation_(2,0) =    0.006704;  
   lidar_to_camera_transformation_(2,1) =    0.072542;   
   lidar_to_camera_transformation_(2,2) =    0.997343;  
   lidar_to_camera_transformation_(2,3) =    0.00000; 
   lidar_to_camera_transformation_(3,0) =    1.332946;  
   lidar_to_camera_transformation_(3,1) =   -4.700044;   
   lidar_to_camera_transformation_(3,2) = -102.268624;  
   lidar_to_camera_transformation_(3,3) =    1.00000;  
 
   //----- CAMERA TO LIDAR TRANSFORMATION (C to L) ----- 
   //----- p_lid = R*p_cam + t = M*p_cam ----- 
   invertMatrix(lidar_to_camera_transformation_,camera_to_lidar_transformation_ );
   
   camera_to_lidar_rotation_(0,0) = camera_to_lidar_transformation_(0,0);
   camera_to_lidar_rotation_(0,1) = camera_to_lidar_transformation_(0,1);
   camera_to_lidar_rotation_(0,2) = camera_to_lidar_transformation_(0,2);
   camera_to_lidar_rotation_(1,0) = camera_to_lidar_transformation_(1,0);
   camera_to_lidar_rotation_(1,1) = camera_to_lidar_transformation_(1,1);
   camera_to_lidar_rotation_(1,2) = camera_to_lidar_transformation_(1,2);
   camera_to_lidar_rotation_(2,0) = camera_to_lidar_transformation_(2,0);
   camera_to_lidar_rotation_(2,1) = camera_to_lidar_transformation_(2,1);
   camera_to_lidar_rotation_(2,2) = camera_to_lidar_transformation_(2,2);
   
   camera_to_lidar_rotation_(0,0) = camera_to_lidar_transformation_(0,0);
   camera_to_lidar_rotation_(0,1) = camera_to_lidar_transformation_(0,1);
   camera_to_lidar_rotation_(0,2) = camera_to_lidar_transformation_(0,2);
   camera_to_lidar_rotation_(1,0) = camera_to_lidar_transformation_(1,0);
   camera_to_lidar_rotation_(1,1) = camera_to_lidar_transformation_(1,1);
   camera_to_lidar_rotation_(1,2) = camera_to_lidar_transformation_(1,2);
   camera_to_lidar_rotation_(2,0) = camera_to_lidar_transformation_(2,0);
   camera_to_lidar_rotation_(2,1) = camera_to_lidar_transformation_(2,1);
   camera_to_lidar_rotation_(2,2) = camera_to_lidar_transformation_(2,2);
   
   fusion_matrix= boost::numeric::ublas::prod(lidar_homogeneous, camera_intrinsics);

   pcl::PointCloud<pcl::PointXYZ>::Ptr global_point_cloud (new pcl::PointCloud<pcl::PointXYZ>);
 
   int W=1920;
   int H=1080;
   cv::Mat depth_ = cv::Mat::zeros(cv::Size(W,H), CV_16UC1);
   ////////////
   //--END--///
   ////////////
   
   pcl::PointXYZ curr_point;
   depth_ = cv::Mat::zeros(cv::Size(W, H), CV_16UC1);
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_li (new pcl::PointCloud<pcl::PointXYZ>);
   global_point_cloud->points.clear();
   
   int posx_ = -1;
   int posy_ = -1;
   float d   = 0;
   uint16_t d_int = 0 ;
    
  
   //--Load single  pdc Lidar image and store it in cloud pointer
   if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_name_listl_[cntl], *cloud_li) == -1){ //* load the file  
      PCL_ERROR ("Couldn't read pcd file  \n");
      //return (-1);
   }
   
   //--Converts a single point from the cloud to meters and also changes the  ROS coordinates into a L3cam lidar 
   //--coordinates, e.g.   ROS(x,y,z)  ->  L3Cam(z,-x,-y)
   for (const auto& point: *cloud_li){   
      lidar_point(0) =  -(float)(1000.0*(point.y));
      lidar_point(1) =  -(float)(1000.0*(point.z));
      lidar_point(2) =   (float)(1000.0*(point.x));
      lidar_point(3) = 1.0;
    
 
    
    //--The projection of the lidar point  into the camera plane in  homogeneous coordinates
   camera_point(0) = ((lidar_point(0)) ? lidar_point(0) * fusion_matrix(0,0) : 0) + ((lidar_point(1)) ? lidar_point(1) * fusion_matrix(1,0) : 0) + ((lidar_point(2)) ? lidar_point(2) * fusion_matrix(2,0) : 0) + fusion_matrix(3,0); 
   camera_point(1) = ((lidar_point(0)) ? lidar_point(0) * fusion_matrix(0,1) : 0) + ((lidar_point(1)) ? lidar_point(1) * fusion_matrix(1,1) : 0) + ((lidar_point(2)) ? lidar_point(2) * fusion_matrix(2,1) : 0) + fusion_matrix(3,1); 
   camera_point(2) = ((lidar_point(0)) ? lidar_point(0) * fusion_matrix(0,2) : 0) + ((lidar_point(1)) ? lidar_point(1) * fusion_matrix(1,2) : 0) + ((lidar_point(2)) ? lidar_point(2) * fusion_matrix(2,2) : 0) + fusion_matrix(3,2); 
    
  
   //--Find the (x,y) position in the camera plane  
   posx_ = camera_point(0)/camera_point(2);
   posy_ = camera_point(1)/camera_point(2);
     
   d = sqrt(pow(lidar_point(0), 2.0) + pow(lidar_point(1), 2.0) + pow(lidar_point(2), 2.0))/1;
   d_int = (uint16_t)d;
   depth_.at<uint16_t>(posy_,posx_) = d_int;
}
  
   //////////////////////////////////////////
   ///////---convert pcl to cvmat ------/////
   //////////////////////////////////////////
   depth_image          = cv::Mat::zeros(H,W,CV_16UC1);
   depth_image_double   = cv::Mat::zeros(H,W,CV_16UC1);
   depth_image_gray     = cv::Mat::zeros(H,W,CV_16UC1);
   depth_image_gray8    = cv::Mat::zeros(H,W,CV_8UC1);
 
   for(int i = 0; i < H; i++){
      for(int j = 0; j < W; j++){   
         if(depth_.at<uint16_t>(i,j) != 0){
            depth_image.row(i).col(j) =  depth_.at<uint16_t>(i,j);
         }    
      }
   }
  
   // Initializing max element as INT_MIN
   uint16_t maxElement = 0;
 
   // checking each element of matrix
   // if it is greater than maxElement,
   // update maxElement 
   for (int i = 0; i < H; i++) {
      for (int j = 0; j < W; j++) {
         if (depth_image.at<uint16_t>(i,j) > maxElement) {
            maxElement = depth_image.at<uint16_t>(i,j);
         }
      }
   }   
    
   for(int i = 0; i < H; i++){
      for(int j = 0; j < W; j++){ 
         if(depth_.at<uint16_t>(i,j) != 0){   
            depth_image_double.at<uint16_t>(i,j)= (depth_image.at<uint16_t>(i,j)/1)*1;  
         }    
      }
   }
   depth_image_double.convertTo(depth_image_double, CV_64F);
    
   if(gray16==true){
      if(gray16_flag==true){
         ROS_ERROR("Gray 16 is ACTIVE");
         gray16_flag=false;
      }
   for(int i = 0; i < H; i++){
      for(int j = 0; j < W; j++){ 
         depth_image_gray.row(i).col(j)    =int16_t((depth_image_double.at<double>(i,j)/float(maxElement))*65536);      
      }
   }
   //Create image color
   cv::Mat image_rgb(H, W, 3); 
   
   // convert the gray image to rgb
   if(depth_image_gray.type()==CV_16UC1) 
      cvtColor(depth_image_gray, image_rgb, CV_GRAY2BGR);
      //cv::imwrite("/path/Lidar16/"  + std::to_string(lidar_cnt*0.000001).substr(8-leading) + ".png", image_rgb);
      //cv:imwrite("/path/laser" + file_cut_name_listlm_[i] + ".jpg", image_rgb);
      lidar_cnt++;       
   }
 
   if(gray8==true){
      if(gray8_flag==true){
         ROS_ERROR("Gray 8 is ACTIVE");
         gray8_flag=false;
      }
   for(int i = 0; i < H; i++){
      for(int j = 0; j < W; j++){ 
         depth_image_gray8.row(i).col(j)    = int((depth_image_double.at<double>(i,j)/float(maxElement))*255);      
      }
   }  
   //Create image color
   cv::Mat image_rgb8(H, W, 3); 

   //convert the gray image to rgb
   if(depth_image_gray8.type()==CV_8UC1) 
      cvtColor(depth_image_gray8, image_rgb8, CV_GRAY2BGR);    
      //cv::imwrite("/path/Lidar/"  + std::to_string(lidar_cnt*0.000001).substr(8-leading) + ".png", image_rgb8);
   
      ///////////////////////////////////////////////////////////
      ///---Calculate the directional vector from the (x,y)   ///
      ///---center point of the calibration board             ///
      ///////////////////////////////////////////////////////////
      Point principal_point = Point(x, y);
      circle(image_rgb8, principal_point, 1, Scalar(0,100,100), 3, LINE_AA);
      circle(image_rgb8, principal_point, 28, Scalar(255,0,255), 3, LINE_AA); 
     
      float principal_pointx = x;
      float principal_pointy = y;
   
      //Get the undirstort point in pixel coordinates
      boost::numeric::ublas::vector<float> uv_und(2);
      uv_und=undistortPoint(principal_pointx, principal_pointy);
    
      //Get the directional vector
      boost::numeric::ublas::vector<float> uv_directional(3);   
      uv_directional=directional_vector(uv_und(0),uv_und(1));
      std::cout << "\n " << "uv_dirx  " << uv_directional(0) << " \n";
      std::cout          << "uv_diry  " << uv_directional(1) << " \n";
      std::cout          << "uv_dirz  " << uv_directional(2) << " \n";
     
      boost::numeric::ublas::vector<float> camara_2lidar_vector_(3);
      boost::numeric::ublas::vector<float> camera_vector_(3);
      
      //Transform a point from camera to lidar 
      camera_vector_(0) = uv_directional(0);
      camera_vector_(1) = uv_directional(1);
      camera_vector_(2) = uv_directional(2);
   
      camara_2lidar_vector_ = boost::numeric::ublas::prec_prod(camera_vector_, camera_to_lidar_rotation_); 
      clx = camara_2lidar_vector_(0);
      cly = camara_2lidar_vector_(1);
      clz = camara_2lidar_vector_(2);
     
      std::cout << "\n " << "clx  " << camara_2lidar_vector_(0) << " \n";
      std::cout          << "cly  " << camara_2lidar_vector_(1) << " \n";
      std::cout          << "clz  " << camara_2lidar_vector_(2) << " \n";
    
     //resize(image_rgb8, image_rgb8, Size(image_rgb8.cols/2, image_rgb8.rows/2)); // to half size or even smaller
     //namedWindow( "Display frame",WINDOW_AUTOSIZE);
     //cv::imshow("initial", image_rgb8);
     //cv::waitKey(0);
      
   }
   
   lidar_cnt++; 
   return {clx,cly,clz};
}
/////////
//-END-//
/////////



/////////////////////////////////////////////////
///---Get the undistort point of the image    ///
/////////////////////////////////////////////////
inline boost::numeric::ublas::vector<float> undistortPoint(float x, float y){
   cv::Mat m_distorted = cv::Mat(1, 2, CV_32F);
   m_distorted.row(0).col(0) = x;
   m_distorted.row(0).col(1) = y;

   cv::Mat m_undistorted = cv::Mat(1, 2, CV_32F);
   cv::undistortPoints(m_distorted, m_undistorted, camera_matrix_, rgb_distortion_coef_);
  
   boost::numeric::ublas::vector<float> uv_undistorted(2);
   uv_undistorted(0) = m_undistorted.at<float>(0, 0) * camera_matrix_.at<float>(0,0) + camera_matrix_.at<float>(0,2);
   uv_undistorted(1) = m_undistorted.at<float>(0, 1) * camera_matrix_.at<float>(1,1) + camera_matrix_.at<float>(1,2);
    
   return uv_undistorted;

}

///////////////////////////////////////
///---Get the directional vector    ///
///////////////////////////////////////
boost::numeric::ublas::vector<float> directional_vector(float dir_x, float dir_y){
   boost::numeric::ublas::vector<float> directional_vector_(3);
   boost::numeric::ublas::vector<float> directional_(3);
      
   directional_(0) = dir_x;
   directional_(1) = dir_y;
   directional_(2) = 1;
      
   directional_vector_ = boost::numeric::ublas::prec_prod(directional_, camera_intrinsics_inv_); 
 
   return directional_vector_;
}

///////////////////////////////////
///---Get the inverse matrix    ///
///////////////////////////////////
inline void invertMatrix(const boost::numeric::ublas::matrix<float> &input, boost::numeric::ublas::matrix<float> &output){
    // Check types and size
    // (pgarcia) ¿ comprobación para que sea estable ?
    assert( input.size1()  == input.size2() && "Input: Only square matrices" );
    assert( output.size1() == output.size2() && "Output: Only square matrices" );
    assert( input.size1()  == output.size2() && "Input and Output must be the same size" );

    switch ( input.size1() ){
    case 3:
    {
        // https://www.thecrazyprogrammer.com/2017/02/c-c-program-find-inverse-matrix.html
        float a = input(0,0);
        float b = input(0,1);
        float c = input(0,2);

        float d = input(1,0);
        float e = input(1,1);
        float f = input(1,2);

        float g = input(2,0);
        float h = input(2,1);
        float i = input(2,2);

        float determinant = a * (e*i - h*f) - d * (b*i - h*c) + g * (b*f - e*c);
        assert(determinant != 0.0 && "Determinant must be nonzero");

        for(int i = 0; i < 3; ++i)
        {
            for(int j = 0; j < 3; ++j)
                output(i,j) = ((input((j+1)%3,(i+1)%3) * input((j+2)%3,(i+2)%3)) - (input((j+1)%3,(i+2)%3) * input((j+2)%3,(i+1)%3))) / determinant;
        }

        break;
    }
    case 4:
    {
        // https://stackoverflow.com/questions/1148309/inverting-a-4x4-matrix (willnode)
        float A2323 = input(2, 2) * input(3, 3) - input(2, 3) * input(3, 2);
        float A1323 = input(2, 1) * input(3, 3) - input(2, 3) * input(3, 1);
        float A1223 = input(2, 1) * input(3, 2) - input(2, 2) * input(3, 1);
        float A0323 = input(2, 0) * input(3, 3) - input(2, 3) * input(3, 0);
        float A0223 = input(2, 0) * input(3, 2) - input(2, 2) * input(3, 0);
        float A0123 = input(2, 0) * input(3, 1) - input(2, 1) * input(3, 0);

        float determinant = input(0, 0) * ( input(1, 1) * A2323 - input(1, 2) * A1323 + input(1, 3) * A1223 )
                - input(0, 1) * ( input(1, 0) * A2323 - input(1, 2) * A0323 + input(1, 3) * A0223 )
                + input(0, 2) * ( input(1, 0) * A1323 - input(1, 1) * A0323 + input(1, 3) * A0123 )
                - input(0, 3) * ( input(1, 0) * A1223 - input(1, 1) * A0223 + input(1, 2) * A0123 );
        assert(determinant != 0.0 && "Determinant must be nonzero");
        determinant = 1 / determinant;

        float A2313 = input(1, 2) * input(3, 3) - input(1, 3) * input(3, 2);
        float A1313 = input(1, 1) * input(3, 3) - input(1, 3) * input(3, 1);
        float A1213 = input(1, 1) * input(3, 2) - input(1, 2) * input(3, 1);
        float A2312 = input(1, 2) * input(2, 3) - input(1, 3) * input(2, 2);
        float A1312 = input(1, 1) * input(2, 3) - input(1, 3) * input(2, 1);
        float A1212 = input(1, 1) * input(2, 2) - input(1, 2) * input(2, 1);
        float A0313 = input(1, 0) * input(3, 3) - input(1, 3) * input(3, 0);
        float A0213 = input(1, 0) * input(3, 2) - input(1, 2) * input(3, 0);
        float A0312 = input(1, 0) * input(2, 3) - input(1, 3) * input(2, 0);
        float A0212 = input(1, 0) * input(2, 2) - input(1, 2) * input(2, 0);
        float A0113 = input(1, 0) * input(3, 1) - input(1, 1) * input(3, 0);
        float A0112 = input(1, 0) * input(2, 1) - input(1, 1) * input(2, 0);

        output(0, 0) = determinant *   ( input(1, 1) * A2323 - input(1, 2) * A1323 + input(1, 3) * A1223 );
        output(0, 1) = determinant * - ( input(0, 1) * A2323 - input(0, 2) * A1323 + input(0, 3) * A1223 );
        output(0, 2) = determinant *   ( input(0, 1) * A2313 - input(0, 2) * A1313 + input(0, 3) * A1213 );
        output(0, 3) = determinant * - ( input(0, 1) * A2312 - input(0, 2) * A1312 + input(0, 3) * A1212 );
        output(1, 0) = determinant * - ( input(1, 0) * A2323 - input(1, 2) * A0323 + input(1, 3) * A0223 );
        output(1, 1) = determinant *   ( input(0, 0) * A2323 - input(0, 2) * A0323 + input(0, 3) * A0223 );
        output(1, 2) = determinant * - ( input(0, 0) * A2313 - input(0, 2) * A0313 + input(0, 3) * A0213 );
        output(1, 3) = determinant *   ( input(0, 0) * A2312 - input(0, 2) * A0312 + input(0, 3) * A0212 );
        output(2, 0) = determinant *   ( input(1, 0) * A1323 - input(1, 1) * A0323 + input(1, 3) * A0123 );
        output(2, 1) = determinant * - ( input(0, 0) * A1323 - input(0, 1) * A0323 + input(0, 3) * A0123 );
        output(2, 2) = determinant *   ( input(0, 0) * A1313 - input(0, 1) * A0313 + input(0, 3) * A0113 );
        output(2, 3) = determinant * - ( input(0, 0) * A1312 - input(0, 1) * A0312 + input(0, 3) * A0112 );
        output(3, 0) = determinant * - ( input(1, 0) * A1223 - input(1, 1) * A0223 + input(1, 2) * A0123 );
        output(3, 1) = determinant *   ( input(0, 0) * A1223 - input(0, 1) * A0223 + input(0, 2) * A0123 );
        output(3, 2) = determinant * - ( input(0, 0) * A1213 - input(0, 1) * A0213 + input(0, 2) * A0113 );
        output(3, 3) = determinant *   ( input(0, 0) * A1212 - input(0, 1) * A0212 + input(0, 2) * A0112 );

        break;
    }
    }

}




