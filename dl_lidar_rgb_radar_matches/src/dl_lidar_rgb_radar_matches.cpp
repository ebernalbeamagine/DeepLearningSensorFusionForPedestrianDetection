
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
 
////////////////////////////////////////////////////////// 
//---This node synchronizes the Lidar, Radar and RGB.---//
//////////////////////////////////////////////////////////

#include <dl_lidar_rgb_radar_matches.h>


//Get the path of all rgb images and cut off the extention
void GetLidarImagesRadarMatches(){
   std::string lidarmin_temp_= "65";
   file_name_listlim_.clear();
   file_name_listlimr_.clear();
   date_listl_.clear();
   date_listr_.clear();
   date_listi_.clear();
   match_listl_.clear();
   match_listr_.clear();
   match_listi_.clear();
   
   int result;
   int pos = 0;
   
   
   for(int i = 0; i < file_name_listl_.size(); i++){
      std::string lidar = file_name_listl_[i];
      std::string lidarmin = file_name_listl_[i];
      std::string lidarsec = file_name_listl_[i];
      //The numbers 60, 62, 19, 58 has to be adapted according to the characters in the path
      //isolate the date yyyymmddhhmmsszz of the laser data
      lidar.erase (lidar.begin()+62, lidar.end()-0);  
      lidar.erase (lidar.begin()+0, lidar.end()-19);
        
      //isolate minutes
      lidarmin.erase (lidarmin.begin()+58, lidarmin.end()-0);  
      lidarmin.erase (lidarmin.begin()+0, lidarmin.end()-2);
     
      //get the data of the first second
      if(lidarmin_temp_!=lidarmin) {  
         lidarmin_temp_=lidarmin;
         date_listl_.push_back(lidar);
      } 
   }
  
   //isolate the date yyyymmddhhmmsszz of the radar data
   for(int i = 0; i < file_name_listr_.size(); i++){
      std::string radar = file_name_listr_[i];
      radar.erase (radar.begin()+62, radar.end()-0);  
      radar.erase (radar.begin()+0, radar.end()-19);
      date_listr_.push_back(radar);
   }
   
   //isolate the date yyyymmddhhmmsszz of the image data
   for(int i = 0; i < file_name_listi_.size(); i++){
      std::string image = file_name_listi_[i];
      image.erase (image.begin()+60, image.end()-0);  
      image.erase (image.begin()+0, image.end()-19);
      date_listi_.push_back(image);   
   }
  
  
   //compare the laser with the image data and get the closest one
   for (int i=0 ; i<date_listl_.size(); i++){
      for (int j=pos ; j<date_listi_.size(); j++){
      
         const char * stringl_ = date_listl_[i].c_str();
         const char * stringi_ = date_listi_[j].c_str();
         
         result = strcmp(stringl_, stringi_);
         pos++;
         if (result < 0){
             file_name_listlim_.push_back(date_listi_[j]);
             break;
         }
       
      }
   }
   
   
   
    //compare the laser-image with the radar data and get the closest one
    pos=0;
    for (int i=0 ; i<file_name_listlim_.size(); i++){
      for (int j=pos ; j<date_listr_.size(); j++){
      
         const char * stringlim_ = file_name_listlim_[i].c_str();
         const char * stringr_  = date_listr_[j].c_str();
         
         result = strcmp(stringlim_, stringr_);
         pos++;
         if (result < 0){
             file_name_listlimr_.push_back(date_listr_[j]);
             break;
         }
         
      }
   }
    
   for (int k=0 ; k<date_listl_.size(); k++){
      match_listl_.push_back(pathl_ + date_listl_[k] + ".pcd");
      match_listi_.push_back(pathi_ + file_name_listlim_[k] + ".png");
      match_listr_.push_back(pathr_  + file_name_listlimr_[k] + ".pcd");
   }
   std::cout<<"Synchronized images:  " << date_listl_.size() << std::endl;
}


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
   return;
}
    
void GetAllLidarFromFolder(std::string folder_pathl_){
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
          if (!strcmp (current_filel_->d_name, ".."))    
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


void GetAllRadarFromFolder(std::string folder_pathr_){
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
      while ((current_filel_ = readdir(FDl_))){
         if(!strcmp (current_filel_->d_name, "."))
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

  
void SaveLidarImagesRadarMatches(){
   leading = 3;
   cnt = 0;
   for (int k=0 ; k<match_listl_.size(); k++){
   
   /////////////////////////////////
   //------Handle the images------//
   ///////////////////////////////// 
   
   cv::Mat image_rgb_(1080, 1920, 3);
   image_rgb_ = cv::imread(match_listi_[k], cv::IMREAD_COLOR);
   
   cv::imwrite("./dl_lidar_rgb_radar_matches/data_syncrhonization/RGB/"  + std::to_string(cnt*0.000001).substr(8-leading) + ".png", image_rgb_);
        
   ////////////////////////////////
   //------Handle the Radar------//
   //////////////////////////////// 
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ra (new pcl::PointCloud<pcl::PointXYZ>);
      
   if(pcl::io::loadPCDFile<pcl::PointXYZ> (match_listr_[k], *cloud_ra) == -1){ //* load the file  
      PCL_ERROR ("Couldn't read pcd file  \n");
   }
 
   if( pcl::io::savePCDFileASCII ("./dl_lidar_rgb_radar_matches/data_syncrhonization/Radar/"  + std::to_string(cnt*0.000001).substr(8-leading) +".pcd", *cloud_ra)==-1){  
      PCL_ERROR ("Couldn't save the pcd Radar file  \n");
      return;
  }
  
   ////////////////////////////////
   //------Handle the Lidar------//
   //////////////////////////////// 
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_li (new pcl::PointCloud<pcl::PointXYZ>);
      
   if(pcl::io::loadPCDFile<pcl::PointXYZ> (match_listl_[k], *cloud_li) == -1){ //* load the file  
      PCL_ERROR ("Couldn't read pcd file  \n");
   }
 
   if( pcl::io::savePCDFileASCII ("./dl_lidar_rgb_radar_matches/data_syncrhonization/Lidar/"  + std::to_string(cnt*0.000001).substr(8-leading) +".pcd", *cloud_li)==-1){  
      PCL_ERROR ("Couldn't save the pcd Lidar file  \n");
      return;
   }
  
   cnt++;
  
   }  

}

int main (int argc, char** argv){  
     ros::init (argc, argv, "dl_lidar_rgb_radar_matches");
     ros::NodeHandle nh_;
     
    system("exec rm -r ./dl_lidar_rgb_radar_matches/data_syncrhonization/Lidar/*"); 
    system("exec rm -r ./dl_lidar_rgb_radar_matches/data_syncrhonization/Radar/*");
    system("exec rm -r ./dl_lidar_rgb_radar_matches/data_syncrhonization//RGB/*");
  
    //! TODO: Specify the path for the images
    pathi_ = "./dl_lid_rad_cam_system/dl_lrgb_ymdhmsz/RGB/";
    GetAllImagesFromFolder(pathi_);
   
    
     //! TODO: Specify the path for the lidar
    pathl_ = "./dl_lid_rad_cam_system/dl_lrgb_ymdhmsz/Lidar/";
    GetAllLidarFromFolder(pathl_);
    
     //! TODO: Specify the path for the radar
    pathr_ = "./dl_lid_rad_cam_system/dl_lrgb_ymdhmsz/Radar/";
    
    GetAllRadarFromFolder(pathr_);
    GetLidarImagesRadarMatches();
    SaveLidarImagesRadarMatches();
    
    
    std::cout <<match_listl_.size()<< "\n";
    std::cout <<match_listi_.size()<< "\n";
    std::cout <<match_listr_.size()<< "\n"<< "\n";
  
      
    ros::Rate loop_rate(100);
    while(ros::ok()){
       ros::spinOnce();
       loop_rate.sleep();
    }
   
    return 0;
  
}
