#include <dl_image_data_simulation.h>

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

int main(int argc, char **argv){
 
  ros::init(argc, argv, "dl_image_data_simulation");

  
  ros::NodeHandle nh_;
  
   camera_matrix_ = cv::Mat(3,3,CV_32F);
   
   rgb_distortion_coef_ = cv::Mat(1,5,CV_32F);
   
   rgb_distortion_coef_.row(0).col(0) =  0.091380;
   rgb_distortion_coef_.row(0).col(1) = -0.150430 ;
   rgb_distortion_coef_.row(0).col(2) =  -0.002450;
   rgb_distortion_coef_.row(0).col(3) =  -0.000005 ;
   rgb_distortion_coef_.row(0).col(4) =   0.000000;
  
  //----- CAMERA INTRINSIC MATRIX [fx 0 0; s fy 0; cx cy 1] ----- 
   camera_matrix_.row(0).col(0) = 1329.715963;//2457.005064; //2475.590928;//m_rgb_camera_intrinsic_matrix(0,0);
   camera_matrix_.row(0).col(1) = 0.000000;//1.375881;//0.126868;
   camera_matrix_.row(0).col(2) = 967.180398; //615.208515;//625.971735;//m_rgb_camera_intrinsic_matrix(2,0);
   camera_matrix_.row(1).col(0) = 0.000000;//0.000000;//0.000000;//m_rgb_camera_intrinsic_matrix(0,1);
   camera_matrix_.row(1).col(1) = 1330.014514; //2450.983886;//2469.070723;//m_rgb_camera_intrinsic_matrix(1,1);
   camera_matrix_.row(1).col(2) = 561.342722;//543.883476; //459.253439;//m_rgb_camera_intrinsic_matrix(2,1);
   camera_matrix_.row(2).col(0) = 0.000000;//0.000000;//0.000000;//m_rgb_camera_intrinsic_matrix(0,2);
   camera_matrix_.row(2).col(1) = 0.000000;//0.000000; //0.000000;//m_rgb_camera_intrinsic_matrix(1,2);
   camera_matrix_.row(2).col(2) = 1.000000;//1.000000;//m_rgb_camera_intrinsic_matrix(2,2);  
 
  //! TODO: Specify the path for the images
    std::string pathi_ = "./data1/RGB/";
   
   
    GetAllImagesFromFolder(pathi_);
  
   size =file_name_listi_.size();
   
   ros::Rate loop_rate(10);
  
   //ros::Rate loop_rate(1);
 
  img_pub_   = nh_.advertise<sensor_msgs::Image>("/image_simulation", 2);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok()){ 
   //////////////////////////////
   ////Handle rgb rect images////
   //////////////////////////////
   
       image = cv::imread(file_name_listi_[count]);
       cv::undistort(image,  image_undistorted, camera_matrix_, rgb_distortion_coef_);
    
       cv_bridge::CvImage img_bridge;
       sensor_msgs::Image img_msg; // >> message to be sent

       std_msgs::Header header; // empty header
       ///header.seq = counter; // user defined counter
       header.stamp = ros::Time::now(); // time
       header.frame_id  =   "lidar";
       img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, image);
       img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
       img_pub_.publish(img_msg); // ros::Publisher pub_img = node.advertise<sensor_msgs::Image>("topic", queuesize);
     
          if(count==(size-1)){
            std::cout<< "\n" << std::endl;
            std::cout<<" ////////////////////////////////////////////////// " << "\n" << std::endl;
            std::cout<<" ////----The Image Node has finished :-) BYE :-) ----//// " << "\n" << std::endl;
            std::cout<<" ////////////////////////////////////////////////// " << "\n" << std::endl;
            ros::shutdown();
       }

       
      /////////////////////////////////////////////////////
      ///---draw all points from radar in the image---/////
      /////////////////////////////////////////////////////

  
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  
   img_pub_.shutdown();
   return 0;
  
}
