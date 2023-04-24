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
//---This node publishes the simulated real time radar data---//
////////////////////////////////////////////////////////////////

#include <dl_radar_data_simulation.h>

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
   
    std::sort(file_name_listr_.begin(), file_name_listr_.end());
    std::cout<<"Total radar files read: "<<file_name_listr_.size()<<std::endl;
    
    return;
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
   radar_cloud_.header.frame_id = "lidar";
   radar_cloud_.header.stamp    = ros::Time::now();                      
   int rr=0;
   
  
    
for (const auto& point: *cloud_ra){
 
   boost::numeric::ublas::vector<float> radar_vector_(4);
    
   radar_vector_(0) =  (float)(1000.0*(point.x));
   radar_vector_(1) =  -(float)(1000.0*(point.y));
   radar_vector_(2) =   (float)(1000.0*(point.z));
   radar_vector_(3) = 1.0;

   radar_point = boost::numeric::ublas::prec_prod(radar_vector_, radar_transformation_); 
    
   radar_cloud_.points[rr].x =  radar_point(0)/1000.0;
   radar_cloud_.points[rr].y =  -radar_point(1)/1000.0;
   radar_cloud_.points[rr].z = radar_point(2)/1000.0;
   rr++;
    
   }
    
   sensor_msgs::PointCloud2 pc2_ra; 
   pc2_ra.header.frame_id = "radar";
   pc2_ra.header.stamp = ros::Time(0); 
          
   sensor_msgs::convertPointCloudToPointCloud2(radar_cloud_,pc2_ra);
   ra_pub_.publish(pc2_ra); 
    
}


int main(int argc, char **argv){
 
   ros::init(argc, argv, "dl_radar_data_simulation");
   ros::NodeHandle nh_;
  
   radar_transformation_.resize(4,4);
  
   radar_transformation_(0,0) =0.99855536;
   radar_transformation_(0,1) =-0.01588738;
   radar_transformation_(0,2) =0.05133007;
   radar_transformation_(0,3) = 0.00000000;
   radar_transformation_(1,0) =0.00908447;
   radar_transformation_(1,1) =0.9914536;
   radar_transformation_(1,2) =0.13014313;
   radar_transformation_(1,3) = 0.00000000;
   radar_transformation_(2,0) =0.05295902;
   radar_transformation_(2,1) =0.12948881;
   radar_transformation_(2,2) = -0.99016564;
   radar_transformation_(2,3) = 0.00000000;
   radar_transformation_(3,0) = 0.0880444;
   radar_transformation_(3,1) =0.09847893;
   radar_transformation_(3,2) = -0.02436598;
   radar_transformation_(3,3) = 1.00000;
  
   //! TODO: Specify the path for the lidar
   std::string pathr_ = "path_to_/data1/Radar/";
   
   GetAllRadarPointCloudFromFolder(pathr_);
 
   ra_pub_  = nh_.advertise<sensor_msgs::PointCloud2>("/radar_simulation", 2); 
    
   size =file_name_listr_.size();
    
   std::cout<<"SIZE  "<<size<<"\n";
  
   ros::Rate loop_rate(18);
  
   int count = 0;
   while (ros::ok()){
   
   //////////////////////////////////////////
   //------Handle the Radar publisher------//
   ////////////////////////////////////////// 

   ros::Time time = ros::Time::now();
   GetAllRadarPointCloud(time,count);
   
   if(count==(size-1)){
            std::cout<<" ////////////////////////////////////////////////// " << "\n" << std::endl;
            std::cout<<" ////----The Radar Node has finished :-) BYE :-) ----//// " << "\n" << std::endl;
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
  
   ra_pub_.shutdown();
   return 0;
  
}
