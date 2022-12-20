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
 
////////////////////////////////////////////////////// 
//---This node takes samples for the calibration.---//
//////////////////////////////////////////////////////
#include <dl_lidar_rgb_radar_sync.h>




//Callback function that makes sensor data readings alignment.
void callback(const  sensor_msgs::PointCloud2ConstPtr& pc2_msg_, const sensor_msgs::ImageConstPtr& img_msg_,  const sensor_msgs::PointCloud2ConstPtr& radar_msg_){

   /////////////////////////////////////////////
   ////////----Lidar ------------/////////////
   ///////////////////////////////////////////
   
   sensor_msgs::PointCloud2 point_cloud2_;
   point_cloud2_.header.frame_id     =  pc2_msg_->header.frame_id;
   point_cloud2_.header.stamp        =  pc2_msg_->header.stamp;
   point_cloud2_.width               =  pc2_msg_->width;
   point_cloud2_.height              =  pc2_msg_->height;
   point_cloud2_.row_step            =  pc2_msg_->row_step;
   point_cloud2_.point_step          =  pc2_msg_->point_step;
  
   point_cloud2_.fields.resize(3);
   point_cloud2_.fields[0].name      =  pc2_msg_->fields[0].name;
   point_cloud2_.fields[0].offset    =  pc2_msg_->fields[0].offset;
   point_cloud2_.fields[0].datatype  =  pc2_msg_->fields[0].datatype;
   point_cloud2_.fields[0].count     =  pc2_msg_->fields[0].count;
 
   point_cloud2_.fields[1].name      =  pc2_msg_->fields[1].name;
   point_cloud2_.fields[1].offset    =  pc2_msg_->fields[1].offset;
   point_cloud2_.fields[1].datatype  =  pc2_msg_->fields[1].datatype;
   point_cloud2_.fields[1].count     =  pc2_msg_->fields[1].count;

   point_cloud2_.fields[2].name      =  pc2_msg_->fields[2].name;
   point_cloud2_.fields[2].offset    =  pc2_msg_->fields[2].offset;
   point_cloud2_.fields[2].datatype  =  pc2_msg_->fields[2].datatype;
   point_cloud2_.fields[2].count     =  pc2_msg_->fields[2].count;
    
   int data_size_  =  pc2_msg_->data.size();
   point_cloud2_.data.resize(data_size_);
 
   int size_ = pc2_msg_->row_step * pc2_msg_->height;
   
   for(int i=0; i<size_; i++ ){     
      point_cloud2_.data[i] = pc2_msg_->data[i]; 
   }
  
   pc2_pub_.publish(point_cloud2_);
    
  
   //////////////////////////////////////////////////////////
   /////--Convert and save Lidar pointcloud2 to pcd---///////
   //////////////////////////////////////////////////////////
   pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud (new pcl::PointCloud<pcl::PointXYZ>);
   pcl::fromROSMsg(point_cloud2_, *lidar_cloud);
  
  
   if(flag_lidar==true){
      if( pcl::io::savePCDFileASCII ("./Lidar/"  + std::to_string(lidar_cnt*0.000001).substr(8-leading) + ".pcd", *lidar_cloud)==-1){  
         PCL_ERROR ("Couldn't save Lidar pcd file  \n");
         return;
      }
      lidar_cnt++;
      flag_lidar=false;
      std::cout << "Lidar has been saved :) " << lidar_cnt <<"\n";
   }
   /////////////////////
   ////---END Lidar--///
   ////////////////////

   ////////////////////////////////////////////////////////////
   /////////////////////////---IMAGE------//////////////////////
   /////////////////////////////////////////////////////////////
   sensor_msgs::Image img_; 
   img_.header.stamp     = img_msg_->header.stamp;
   img_.header.frame_id  = img_msg_->header.frame_id;
   img_.width            = img_msg_->width; 
   img_.height           = img_msg_->height; 
   img_.encoding         = img_msg_->encoding;
   img_.is_bigendian     = img_msg_->is_bigendian; 
   img_.step             = img_msg_->step;
   
   int img_size_  =  img_msg_->data.size();
   img_.data.resize(img_size_);
   for(int i=0; i<img_size_; i++ ){     
      img_.data[i] = img_msg_->data[i];
   }
  
   img_pub_.publish(img_);
    
   //Convert ROS iamge to cv image and save it into memory.
   cv_bridge::CvImagePtr cv_img_;
   try
   {
     cv_img_ = cv_bridge::toCvCopy(img_, sensor_msgs::image_encodings::BGR8);
   }
   catch (cv_bridge::Exception& e)
   {
     ROS_ERROR("cv_bridge exception: %s", e.what());
     return;
   }
   if(flag_rgb==true){ 
      cv:imwrite("./RGB/" + std::to_string(cnt*0.000001).substr(8-leading) + ".png", cv_img_->image);
      cnt++;
      flag_rgb=false;
      std::cout << "RGB has benn saved :) " << lidar_cnt <<"\n";
   }
   /////////////////////////////////////////////////////
   ////////////////---END IMAGE---//////////////////////
   /////////////////////////////////////////////////////
   
   ///////////////////////////////////////////////////// 
   ///////////////////----Radar---//////////////////////
   /////////////////////////////////////////////////////
   sensor_msgs::PointCloud2 radar_cloud2_;
   radar_cloud2_.header.frame_id  =  radar_msg_->header.frame_id;
   radar_cloud2_.header.stamp     =  radar_msg_->header.stamp;
   radar_cloud2_.width            =  radar_msg_->width;
   radar_cloud2_.height           =  radar_msg_->height;
   radar_cloud2_.row_step         =  radar_msg_->row_step;
   radar_cloud2_.point_step       =  radar_msg_->point_step;
  
   radar_cloud2_.fields.resize(6);
   radar_cloud2_.fields[0].name      =  radar_msg_->fields[0].name;
   radar_cloud2_.fields[0].offset    =  radar_msg_->fields[0].offset;
   radar_cloud2_.fields[0].datatype  =  radar_msg_->fields[0].datatype;
   radar_cloud2_.fields[0].count     =  radar_msg_->fields[0].count;
 
   radar_cloud2_.fields[1].name      =  radar_msg_->fields[1].name;
   radar_cloud2_.fields[1].offset    =  radar_msg_->fields[1].offset;
   radar_cloud2_.fields[1].datatype  =  radar_msg_->fields[1].datatype;
   radar_cloud2_.fields[1].count     =  radar_msg_->fields[1].count;

   radar_cloud2_.fields[2].name      =  radar_msg_->fields[2].name;
   radar_cloud2_.fields[2].offset    =  radar_msg_->fields[2].offset;
   radar_cloud2_.fields[2].datatype  =  radar_msg_->fields[2].datatype;
   radar_cloud2_.fields[2].count     =  radar_msg_->fields[2].count;
 
   radar_cloud2_.fields[3].name      =  radar_msg_->fields[3].name;
   radar_cloud2_.fields[3].offset    =  radar_msg_->fields[3].offset;
   radar_cloud2_.fields[3].datatype  =  radar_msg_->fields[3].datatype;
   radar_cloud2_.fields[3].count     =  radar_msg_->fields[3].count;
   
   radar_cloud2_.fields[4].name      =  radar_msg_->fields[4].name;
   radar_cloud2_.fields[4].offset    =  radar_msg_->fields[4].offset;
   radar_cloud2_.fields[4].datatype  =  radar_msg_->fields[4].datatype;
   radar_cloud2_.fields[4].count     =  radar_msg_->fields[4].count;
   
   radar_cloud2_.fields[5].name      =  radar_msg_->fields[5].name;
   radar_cloud2_.fields[5].offset    =  radar_msg_->fields[5].offset;
   radar_cloud2_.fields[5].datatype  =  radar_msg_->fields[5].datatype;
   radar_cloud2_.fields[5].count     =  radar_msg_->fields[5].count;

   int radar_size_  =  radar_msg_->data.size();
   radar_cloud2_.data.resize(radar_size_);
 
   int rsize_ = radar_msg_->row_step * radar_msg_->height;
 
   for(int i=0; i<rsize_; i++ ){     
      radar_cloud2_.data[i] = radar_msg_->data[i]; 
   }

   radar_pub_.publish(radar_cloud2_);

   /////--Convert and save Radar pointcloud2 to pcd---///////
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_radar (new pcl::PointCloud<pcl::PointXYZ>);
   pcl::fromROSMsg(radar_cloud2_, *cloud_radar);
  
   if(flag_radar==true){
      if( pcl::io::savePCDFileASCII ("./Radar/"  + std::to_string(radar_cnt*0.000001).substr(8-leading) + ".pcd", *cloud_radar)==-1){  
         PCL_ERROR ("Couldn't save pcd Radar file  \n");
         return;
      }
      radar_cnt++;
      flag_radar= false;
      std::cout << "radar has benn saved :) " << radar_cnt <<"\n";
   }  
   /////////////////////////    
   ////---END Radar---//////   
   /////////////////////////

}

bool service_callback(lidar_rgb_radar_sync::process::Request  &req, lidar_rgb_radar_sync::process::Response &res){
   std::stringstream c;
   c<<req.in.c_str();
   std::stringstream ss;

   if(c.str()=="pause"){
      flag_process=false;
      flag_lidar = false;
      flag_radar = false;
      flag_rgb   = false;
      ss << "OK paused";
      res.out = ss.str();
      ROS_ERROR("PROCESS PAUSED");
      ROS_ERROR("From Client  [%s], Server says [%s]",req.in.c_str(),res.out.c_str());}
   if(c.str()=="start"){
      flag_lidar = true;
      flag_radar = true;
      flag_rgb   = true;
      ss << "OK start";
      res.out = ss.str();
      ROS_ERROR("PROCESS START ONCE");
      ROS_ERROR("From Client  [%s], Server says [%s]",req.in.c_str(),res.out.c_str());}

   return true;
}


int main (int argc, char** argv){  
   ros::init (argc, argv, "dl_lidar_rgb_radar_sync");
   ros::NodeHandle nh_;
    
   std::cout << "lidar_rgb_radar_sync" << "\n";
    //system("exec rm -r ./Radar/*");
    //system("exec rm -r ./Lidar/*");
    //system("exec rm -r ./RGB/*");
   
   message_filters::Subscriber<sensor_msgs::PointCloud2> laser_sub_(nh_, "/PC2_lidar", 10);
   message_filters::Subscriber<sensor_msgs::Image> rgbd_sub_(nh_,  "/img_rgb_", 10); 
   message_filters::Subscriber<sensor_msgs::PointCloud2> radar_sub_(nh_, "/radar/target_list_cartesian", 10);
     
   typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2,sensor_msgs::Image,sensor_msgs::PointCloud2> MySyncPolicy;
   Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),  laser_sub_, rgbd_sub_, radar_sub_);
   sync.registerCallback(boost::bind(&callback, _1, _2, _3));


   // setting up publishers
   pc2_pub_      = nh_.advertise<sensor_msgs::PointCloud2>("/lidar_sync", 2);
   radar_pub_    = nh_.advertise<sensor_msgs::PointCloud2>("/radar_sync", 2);
   img_pub_      = nh_.advertise<sensor_msgs::Image>("/img_sync", 2);
   
   ROS_INFO("Ready to receive from client.	"); 
   service = nh_.advertiseService("action", service_callback);
 
   ros::Rate loop_rate(10);
   while(ros::ok()){
      ros::spinOnce();
      loop_rate.sleep();
   }

   
   // clean up subscribers and publishers 
   pc2_pub_.shutdown();
   img_pub_.shutdown();
   radar_pub_.shutdown();
   return 0;
}
