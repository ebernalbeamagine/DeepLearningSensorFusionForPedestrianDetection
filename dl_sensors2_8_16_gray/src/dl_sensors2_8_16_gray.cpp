/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C)  
 *
 * This file is part of software developed by UPC-Cd6-Beamagin group.
 *
 * Author: Alfredo Chávez Plascencia  alfredo.chavez@upc.edu
 * Author: Pablo García Gómez        pablo.garcia@beamagine.com
 * Author: Eduardo Bernal Perez      eduardo.bernal@upc.edu
 * 
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this file.  If not, see <http://www.gnu.org/licenses/>.
 */
 
/////////////////////////////////////////////////////////////////////////////////////////////////////////// 
//---This node takes the lidar and radar pointcloud and normilize them either into 8 or 16-depth bits.---//
///////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <dl_sensors2_8_16_gray.h>



//Get the path of all lidar images and cut off the extention  
void GetAllImagesFromFolder(std::string folder_pathi_){
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
      while ((current_filei_ = readdir(FDi_))) {
         if (!strcmp (current_filei_->d_name, "."))
            continue;
               if (!strcmp (current_filei_->d_name, ".."))    
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
   return;
}
    
void GetAllLidarPointCloudFromFolder(std::string folder_pathl_){
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
               full_pathl_ = folder_pathl_ + current_file_namel_;
               file_name_listl_.push_back(full_pathl_);  
            }
         }
      }
      if(closedir(FDl_)==0)
         std::cout<<"The dir FDl_ is closed successfully"<<std::endl;    
   }
   
   //!Ordernar los ficheros 
   std::sort(file_name_listl_.begin(), file_name_listl_.end());
   std::cout<<"Total laser files read: "<<file_name_listl_.size()<<std::endl;
    
   return;
}


void GetAllRadarPointCloudFromFolder(std::string folder_pathr_){
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
      while ((current_filel_ = readdir(FDl_))) {
         if (!strcmp (current_filel_->d_name, "."))
            continue;
         if (!strcmp (current_filel_->d_name, ".."))    
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
    
   return;
}




int GetAllLidarPointCloud(ros::Time time, int cntl){  
   pcl::PointXYZ curr_point;
   depth_ = cv::Mat::zeros(cv::Size(W, H), CV_16UC1);
   pos_coordinate_xyz_.clear();
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_li (new pcl::PointCloud<pcl::PointXYZ>);
   global_point_cloud->points.clear();
   
   int posx_=-1;
   int posy_=-1;
   float d = 0;
   uint16_t d_int = 0 ;
 
   if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_name_listl_[cntl], *cloud_li) == -1){ //* load the file  
      PCL_ERROR ("Couldn't read pcd file  \n");
      return (-1);
   }
   //std::cout << "Loaded " << cloud_li->width * cloud_li->height << " data points from test_pcd.pcd with the following fields: " << std::endl;
   
   
   
   sensor_msgs::PointCloud cloud_;
   cloud_.points.resize(cloud_li->size ());
   cloud_.header.frame_id = "lidar";
   cloud_.header.stamp    = ros::Time::now();       
   int kk=0;
   for (const auto& point: *cloud_li){
      lidar_point(0) =  -(float)(1000.0*(point.y));
      lidar_point(1) =  -(float)(1000.0*(point.z));
      lidar_point(2) =   (float)(1000.0*(point.x));
      lidar_point(3) = 1.0;
 
      cloud_.points[kk].x = point.x;
      cloud_.points[kk].y = point.y;
      cloud_.points[kk].z = point.z;

      camera_point(0) = ((lidar_point(0)) ? lidar_point(0) * fusion_matrix(0,0) : 0) + ((lidar_point(1)) ? lidar_point(1) * fusion_matrix(1,0) : 0) + ((lidar_point(2)) ? lidar_point(2) * fusion_matrix(2,0) : 0) + fusion_matrix(3,0); //lidar_point(3) * fusion_matrix(3,0);
      camera_point(1) = ((lidar_point(0)) ? lidar_point(0) * fusion_matrix(0,1) : 0) + ((lidar_point(1)) ? lidar_point(1) * fusion_matrix(1,1) : 0) + ((lidar_point(2)) ? lidar_point(2) * fusion_matrix(2,1) : 0) + fusion_matrix(3,1); //lidar_point(3) * fusion_matrix(3,1);
      camera_point(2) = ((lidar_point(0)) ? lidar_point(0) * fusion_matrix(0,2) : 0) + ((lidar_point(1)) ? lidar_point(1) * fusion_matrix(1,2) : 0) + ((lidar_point(2)) ? lidar_point(2) * fusion_matrix(2,2) : 0) + fusion_matrix(3,2); //lidar_point(3) * fusion_matrix(3,2);
 
      posx_ = camera_point(0)/camera_point(2);
      posy_ = camera_point(1)/camera_point(2);
     
      d = sqrt(pow(lidar_point(0), 2.0) + pow(lidar_point(1), 2.0) + pow(lidar_point(2), 2.0))/1;
      d_int = (uint16_t)d;
      depth_.at<uint16_t>(posy_,posx_) = d_int;
      kk++;
   }
 
   sensor_msgs::PointCloud2 pc2_; 
   pc2_.header.frame_id = "lidar";
   pc2_.header.stamp = ros::Time(0); 
          
   sensor_msgs::convertPointCloudToPointCloud2(cloud_,pc2_);
   li_pub_.publish(pc2_);  
   
   /////////////////////////////////////////////////////////////////
   ///////---convert pcl to cvmat and save it as a image jpg---////
   //////////////////////////////////////////////////////////////// 
   depth_image          = cv::Mat::zeros(H,W,CV_16UC1);
   depth_image_double   = cv::Mat::zeros(H,W,CV_16UC1);
   depth_image_gray     = cv::Mat::zeros(H,W,CV_16UC1);
   depth_image_gray8    = cv::Mat::zeros(H,W,CV_8UC1);
 
 
   for(int i = 0; i < H; i++){
      for(int j = 0; j < W; j++){   
         if(depth_.at<uint16_t>(i,j) != 0){
            depth_image.row(i).col(j) =  depth_.at<uint16_t>(i,j);
            //std::cout << " " << depth_image.row(i).col(j);
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
             // std::cout << " " << depth_image_double.at<uint16_t>(i,j);        
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
      // std::cout << " " <<      depth_image_gray.row(i).col(j);
      }
   }
   //Create image color
   cv::Mat image_rgb(H, W, 3); 

   // convert the gray image to rgb
   if (depth_image_gray.type()==CV_16UC1) 
      cvtColor(depth_image_gray, image_rgb, CV_GRAY2BGR);
        
   cv::imwrite("../sensor_fusion_cnn/data_rotation_translation/data_R_matrix/data2interpolate/Lidar/"  + std::to_string(lidar_cnt*0.000001).substr(8-leading) + ".png", image_rgb);

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
         //std::cout << " " <<      depth_image_gray.row(i).col(j);
       }
   }  
    
   //Create image color
   cv::Mat image_rgb8(H, W, 3); 

   // convert the gray image to rgb
   if (depth_image_gray8.type()==CV_8UC1) 
      cvtColor(depth_image_gray8, image_rgb8, CV_GRAY2BGR);    
   
   cv::imwrite("../sensor_fusion_cnn/data_rotation_translation/data_R_matrix/data2interpolate/Lidar/"  + std::to_string(lidar_cnt*0.000001).substr(8-leading) + ".png", image_rgb8);

   Point origin = Point(991.506,528.494);
   circle(image_rgb8, origin, 1, Scalar(0,100,100), 3, LINE_AA);
   circle(image_rgb8, origin, 28, Scalar(255,0,255), 3, LINE_AA); 
 
   float pointxx  = (31.5  * 0.00075) -0.72736;    
   float pointyy  = (11.58  * 0.00075) -0.42206;
   
   lidar_cnt++;
      
   }
   
   
   return 0;
}

int GetAllRadarPointCloud(ros::Time time, int cntr){ 
   pcl::PointXYZ radar_curr_point;
   depthr_ = cv::Mat::zeros(cv::Size(W, H), CV_16UC1);
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ra (new pcl::PointCloud<pcl::PointXYZ>);
  
   int r_posx_=-1;
   int r_posy_=-1;
   float dr = 0;
   uint16_t dr_int = 0 ;

   if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_name_listr_[cntr], *cloud_ra) == -1){ //* load the file  
      PCL_ERROR ("Couldn't read radar pcd file  \n");
      return (-1);
   }
   
   sensor_msgs::PointCloud radar_cloud_;
   radar_cloud_.points.resize(cloud_ra->size ());
   radar_cloud_.header.frame_id = "radar";
   radar_cloud_.header.stamp    = ros::Time::now();                      
   int rr=0;
   
   rad_img_x.clear();
   rad_img_y.clear();
    
   for (const auto& point: *cloud_ra){
      boost::numeric::ublas::vector<float> radar_vector_(4);
      radar_vector_(0) =  -(float)(1000.0*(point.y));
      radar_vector_(1) =  -(float)(1000.0*(point.z));
      radar_vector_(2) =   (float)(1000.0*(point.x));
      radar_vector_(3) = 1.0;

      radar_point = boost::numeric::ublas::prec_prod(radar_vector_, radar_transformation_); 

      radar_cloud_.points[rr].x =  radar_point(2)/1000.0;
      radar_cloud_.points[rr].y = -radar_point(0)/1000.0;
      radar_cloud_.points[rr].z = -radar_point(1)/1000.0;
      rr++;
 
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
     
      rad_img_x.push_back(r_posx_);
      rad_img_y.push_back(r_posy_);
   
      depthr_.at<uint16_t>(r_posy_,r_posx_) = dr_int;
  
      for(int i = 0; i < H; i++){
         int j = r_posx_;
         depthr_.at<uint16_t>(i,j)= dr_int;
      } 
    
   
   }
   
   sensor_msgs::PointCloud2 pc2_ra; 
   pc2_ra.header.frame_id = "radar";
   pc2_ra.header.stamp = ros::Time(0); 
          
   sensor_msgs::convertPointCloudToPointCloud2(radar_cloud_,pc2_ra);

   ra_pub_.publish(pc2_ra); 
  
   /////////////////////////////////////////////////////////////////
   ///////---convert pcl to cvmat and save it as a image jpg---////
   //////////////////////////////////////////////////////////////// 
   depthr_image          = cv::Mat::zeros(H,W,CV_16UC1);
   depthr_image_double   = cv::Mat::zeros(H,W,CV_16UC1);
   depthr_image_gray     = cv::Mat::zeros(H,W,CV_16UC1);
   depthr_image_gray8    = cv::Mat::zeros(H,W,CV_8UC1);

   for(int i = 0; i < H; i++){
      for(int j = 0; j < W; j++){   
         if(depthr_.at<uint16_t>(i,j) != 0){
            depthr_image.row(i).col(j) =  depthr_.at<uint16_t>(i,j);
             // std::cout << " " << depthr_.at<uint16_t>(i,j);
         }    
       }
   }

   // Initializing max element as INT_MIN
   uint16_t maxElement = 0;
 
   // checking each element of matrix
   // if it is greater than maxElement,
   // update maxElement 
   for(int i = 0; i < H; i++) {
      for(int j = 0; j < W; j++) {
         if(depthr_image.at<uint16_t>(i,j) > maxElement) {
            maxElement = depthr_image.at<uint16_t>(i,j);
            // std::cout <<"\n "<< " maxElement " << maxElement<<"\n";   
         }
      }
   }
  
   for(int i = 0; i < H; i++){
      for(int j = 0; j < W; j++){ 
         if(depthr_.at<uint16_t>(i,j) != 0){
             //depth_image_double.at<uint16_t>(i,j)= (depth_image_double.at<uint16_t>(i,j)/maxElement)*255;
             //depth_image_double.at<uint16_t>(i,j)= (depth_image_double.at<uint16_t>(i,j)/maxElement)*255;      
             depthr_image_double.at<uint16_t>(i,j)= (depthr_image.at<uint16_t>(i,j)/1)*1;  
             // std::cout << " " << depth_image_double.at<uint16_t>(i,j);        
           }    
       }
   }
    
   depthr_image_double.convertTo(depthr_image_double, CV_64F);

   for(int i = 0; i < H; i++){
      for(int j = 0; j < W; j++){ 
         depthr_image_gray8.row(i).col(j)    = int((depthr_image_double.at<double>(i,j)/float(maxElement))*65536);      
           //   std::cout << " " <<      depth_image_gray.row(i).col(j);
      }
   }  
    
   //Create image color
   cv::Mat imager_rgb8(H, W, 3); 

   // convert the gray image to rgb
   if (depthr_image_gray8.type()==CV_8UC1) 
      cvtColor(depthr_image_gray8, imager_rgb8, CV_GRAY2BGR);    

   cv::imwrite("../sensor_fusion_cnn/data_rotation_translation/data_R_matrix/data2interpolate/Radar/"  + std::to_string(radar_cnt*0.000001).substr(8-leading) + ".png", imager_rgb8);    
     

   
   
   radar_cnt++; 
   //std::cout<<"radar_cnt  "<<  radar_cnt<<"\n";   
   return 0;

}

 


int main(int argc, char **argv){
   ros::init(argc, argv, "dl_sensors2_8_16_gray");
   ros::NodeHandle nh_;
  
   system("exec rm -r ../sensor_fusion_cnn/data_rotation_translation/data_R_matrix/data2interpolate/Lidar/*"); 
   system("exec rm -r ../sensor_fusion_cnn/data_rotation_translation/data_R_matrix/data2interpolate/Radar/*"); 
   system("exec rm -r ../sensor_fusion_cnn/data_rotation_translation/data_R_matrix/data2interpolate/RGB/*"); 
  

   nh_.param("/gray16", gray16, false);
   nh_.param("/gray8",  gray8,  false);
  
   std::cout<<"gray16 "<<gray16<<"\n";
   std::cout<<"gray8  "<<gray8<<"\n";
 
   radar_transformation_.resize(4,4);
   lidar_homogeneous.resize(4,3);
   fusion_matrix.resize(4,3);
   camera_intrinsics.resize(3,3);
   
   camera_matrix_ = cv::Mat(3,3,CV_32F);
   rgb_distortion_coef_ = cv::Mat(1,5,CV_32F);

   radar_transformation_(0,0) =  0.99855536;
   radar_transformation_(0,1) = -0.01588738;  
   radar_transformation_(0,2) =  0.05133007;
   radar_transformation_(0,3) =  0.00000000;
   radar_transformation_(1,0) =  0.00908447;
   radar_transformation_(1,1) =  0.9914536;
   radar_transformation_(1,2) =  0.13014313;
   radar_transformation_(1,3) =  0.00000000;
   radar_transformation_(2,0) =  0.05295902;
   radar_transformation_(2,1) =  0.12948881;
   radar_transformation_(2,2) = -0.99016564;
   radar_transformation_(2,3) =  0.00000000;
   radar_transformation_(3,0) =  0.0880444;
   radar_transformation_(3,1) =  0.09847893;
   radar_transformation_(3,2) = -0.02436598;
   radar_transformation_(3,3) =  1.00000;
  
  
   rgb_distortion_coef_.row(0).col(0) =  0.091380;
   rgb_distortion_coef_.row(0).col(1) = -0.150430;
   rgb_distortion_coef_.row(0).col(2) = -0.002450;
   rgb_distortion_coef_.row(0).col(3) = -0.000005;
   rgb_distortion_coef_.row(0).col(4) =  0.000000;
   
   
 
   //----- CAMERA INTRINSIC MATRIX [fx 0 0; s fy 0; cx cy 1] ----- 
   camera_matrix_.row(0).col(0) = 1329.715963;
   camera_matrix_.row(0).col(1) =    0.000000;
   camera_matrix_.row(0).col(2) =  967.180398; 
   camera_matrix_.row(1).col(0) =    0.000000;
   camera_matrix_.row(1).col(1) = 1330.014514; 
   camera_matrix_.row(1).col(2) =  561.342722;
   camera_matrix_.row(2).col(0) =    0.000000;
   camera_matrix_.row(2).col(1) =    0.000000;
   camera_matrix_.row(2).col(2) =    1.000000;  
   
   
  
   //----- CAMERA INTRINSIC MATRIX [fx 0 0; s fy 0; cx cy 1] -----
   camera_intrinsics(0,0) =  1329.715963;
   camera_intrinsics(0,1) =     0.000000;
   camera_intrinsics(0,2) =     0.000000;
   camera_intrinsics(1,0) =     0.000000;
   camera_intrinsics(1,1) =  1330.014514;
   camera_intrinsics(1,2) =     0.000000;
   camera_intrinsics(2,0) =   967.180398;
   camera_intrinsics(2,1) =   561.342722;
   camera_intrinsics(2,2) =     1.000000;
   
  
   
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

 
   fusion_matrix= boost::numeric::ublas::prod(lidar_homogeneous, camera_intrinsics);

   //! TODO: Specify the path for the images
   std::string pathi_ = "../dl_lidar_rgb_radar_matches/data_syncrhonization/RGB/";

   GetAllImagesFromFolder(pathi_);
     
   //! TODO: Specify the path for the lidar
   std::string pathl_ = "../dl_lidar_rgb_radar_matches/data_syncrhonization/Lidar/";
      
   GetAllLidarPointCloudFromFolder(pathl_);
     
   //! TODO: Specify the path for the radar
   std::string pathr_ = "../dl_lidar_rgb_radar_matches/data_syncrhonization/Radar/";
  
   GetAllRadarPointCloudFromFolder(pathr_);
     
   size =file_name_listl_.size();
   
   chatter_pub = nh_.advertise<std_msgs::String>("chatter", 10);
   li_pub_      = nh_.advertise<sensor_msgs::PointCloud2>("/lidar_gray", 2); 
   ra_pub_      = nh_.advertise<sensor_msgs::PointCloud2>("/radar_gray", 2); 
   
   ros::Rate loop_rate(1000);

   int count = 0;
   while (ros::ok()){ 
      std_msgs::String msg;
      std::stringstream ss;
      ss << "hello world " << count;
      msg.data = ss.str();
      chatter_pub.publish(msg);
    
      //////////////////////////////////////////
      //------Handle the Lidar publisher------//
      ////////////////////////////////////////// 
   
      ros::Time time = ros::Time::now();
      GetAllLidarPointCloud(time,count);
  
      ///////////
      //--END--//
      //////////
   
   
  
      //////////////////////////////////////////
      //------Handle the Radar publisher------//
      ////////////////////////////////////////// 
   
      GetAllRadarPointCloud(time,count);
   
      ///////////
      //--END--//
      //////////
   
      //////////////////////////////
      ////Handle rgb rect images////
      //////////////////////////////
   
        /*image = cv::imread(file_name_listi_[count]);
        cv::undistort(image,  image_undistorted, camera_matrix_, rgb_distortion_coef_);
       
      /////////////////////////////////////////////////////
      ///---draw all points from radar in the image---/////
      /////////////////////////////////////////////////////
      // vector<Vec3f> circles;
      
      
     
      for( size_t i = 0; i < rad_img_x.size(); i++ ){ 
         circle(image_undistorted, Point(rad_img_x[i], rad_img_y[i]),28, Scalar(255,0,0),3, LINE_AA);
         circle(image_undistorted, Point(rad_img_x[i], rad_img_y[i]), 1, Scalar(0,0,255), 3, LINE_AA);
    }
 
      std::cout<<"image  " << count<< std::endl;    
      resize(image_undistorted, image_undistorted, Size(image_undistorted.cols/2, image_undistorted.rows/2)); // to half size or even smaller
      namedWindow("Display Image", WINDOW_AUTOSIZE );
      imshow("Display Image", image_undistorted);
      waitKey(0); */
      // cv::imwrite("/home/acp/undistorted.png", image_undistorted);
      //getchar();
       
     
      ///////////
      //--END--//
      ///////////
    
      // cv::imwrite("/home/acp/catkin_ws/src/sensors2_8_16_gray/dataset2/RGB/"  + std::to_string(*0.000001).substr(8-leading) + ".png", image);
      
     // cv::imwrite("/home/acp/catkin_ws/src/err_opt_lid_rad/data1/RGB/"  + std::to_string(img_cnt*0.000001).substr(8-leading) + ".png", image);
      
      // cv::imwrite("/home/acp/Downloads/cnn/encdec/sensor_fusion_cnn/dataset1_lidar_radar_rgb/data2interpolate/RGB/"  + std::to_string(img_cnt*0.000001).substr(8-leading) + ".png", image);
       
     
       
       img_cnt++;
       
       
       
   ///////////
   //--END--//
   ///////////
   
   std::cout<<"image  " << count<< std::endl;
   ++count;
    
     
      if(count==size){
         std::cout<< "\n" << std::endl;
         std::cout<<" ////////////////////////////////////////////////// " << "\n" << std::endl;
         std::cout<<" ////----The Node has finished :-) BYE :-) ----//// " << "\n" << std::endl;
         std::cout<<" ////////////////////////////////////////////////// " << "\n" << std::endl;
         ros::shutdown(); 
      }
      ros::spinOnce();
      loop_rate.sleep();
   
   }


   chatter_pub.shutdown();
   li_pub_.shutdown();
   ra_pub_.shutdown();
   return 0;
}
