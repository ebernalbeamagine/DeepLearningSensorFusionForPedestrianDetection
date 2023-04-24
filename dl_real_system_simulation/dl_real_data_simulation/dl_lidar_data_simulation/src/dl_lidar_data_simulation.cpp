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
 
//////////////////////////////////////////////////////////////// 
//---This node publishes the simulated real time lidar data---//
////////////////////////////////////////////////////////////////


#include <dl_lidar_data_simulation.h>


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

   sensor_msgs::PointCloud cloud_;
   cloud_.points.resize(cloud_li->size ());
   cloud_.header.frame_id = "lidar";
   cloud_.header.stamp    = ros::Time::now();       
   int kk=0;
  for (const auto& point: *cloud_li){
 
    cloud_.points[kk].x = point.x;
    cloud_.points[kk].y = point.y;
    cloud_.points[kk].z = point.z;
    
    kk++;
    
    }
    
    
   sensor_msgs::PointCloud2 pc2_; 
   pc2_.header.frame_id = "lidar";
   pc2_.header.stamp = ros::Time(0); 
          
   sensor_msgs::convertPointCloudToPointCloud2(cloud_,pc2_);
   li_pub_.publish(pc2_); 
    
}

int main(int argc, char **argv){
 
  ros::init(argc, argv, "dl_lidar_data_simulation");

  ros::NodeHandle nh_;
  
  //! TODO: Specify the path for the lidar
      std::string pathl_ = "/path_to_/data1/Lidar/";
   
    GetAllLidarPointCloudFromFolder(pathl_);
 
    li_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/lidar_simulation", 2); 
    
    size =file_name_listl_.size();
 
    ros::Rate loop_rate(6);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok()){
 
   //////////////////////////////////////////
   //------Handle the Lidar publisher------//
   ////////////////////////////////////////// 
   
   ros::Time time = ros::Time::now();
   GetAllLidarPointCloud(time,count);
   
   if(count==(size-1)){
        // std::cout<<"lidar_size  " << count << "\n";
        // count=0;
        std::cout<< "\n" << std::endl;
            std::cout<<" ////////////////////////////////////////////////// " << "\n" << std::endl;
            std::cout<<" ////----The Lidar Node has finished :-) BYE :-) ----//// " << "\n" << std::endl;
            std::cout<<" ////////////////////////////////////////////////// " << "\n" << std::endl;
            ros::shutdown(); 
       
       }

   ///////////
   //--END--//
   //////////

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  
   li_pub_.shutdown();
   return 0;
  
}
