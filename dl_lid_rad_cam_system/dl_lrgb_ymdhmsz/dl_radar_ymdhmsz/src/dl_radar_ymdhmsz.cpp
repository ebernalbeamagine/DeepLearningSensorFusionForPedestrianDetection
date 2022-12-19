/*************************************************************************************
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
 
///////////////////////////////////////////////////////////////////////////////////////////// 
//---This node receives the radar pointcloud and save it as the format yyMMdd-hhmmss-zz.---//
/////////////////////////////////////////////////////////////////////////////////////////////

#include <dl_radar_ymdhmsz.h>


void  RadarCallback(const  sensor_msgs::PointCloud2ConstPtr& radar_msg_){ 
   pcl::PCLPointCloud2 radar_pcl_pc2;
   pcl_conversions::toPCL(*radar_msg_,radar_pcl_pc2);
   pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
   pcl::fromPCLPointCloud2(radar_pcl_pc2,*temp_cloud);
   
   time_t t = time(NULL);
   struct tm time = *localtime(&t);
   uint16_t year;
   uint8_t month, day, hour, min, sec;
   year = (uint16_t) time.tm_year + 1900;
   month = (uint8_t)time.tm_mon + 1;
   day = (uint8_t) time.tm_mday;
   hour = (uint8_t) time.tm_hour;
   min = (uint8_t) time.tm_min;
   sec = (uint8_t) time.tm_sec;
    
   struct timeval time_now{};
   gettimeofday(&time_now, nullptr);
   time_t msecs_time = (time_now.tv_sec * 1000) + (time_now.tv_usec / 1000);

   int msec =  msecs_time % 1000;
  
   char md [50];
   sprintf (md, "%02d%02d", month,day);
  
   char hms [50];
   sprintf (hms, "%02d%02d%02d", hour,min,sec);
     
   char ms [50];
   sprintf (ms, "%03d", msec);
    
   if(flag_radar==true){    
      if( pcl::io::savePCDFileASCII ("../lrgb_ymdhmsz/Radar/"  + std::to_string(year)+ md + "_" + hms + "_" + ms+".pcd", *temp_cloud)==-1){  
        PCL_ERROR ("Couldn't save pcd Radar file  \n");
        return;
      }
   }
   
   rad_pub_.publish(radar_msg_);
}

bool service_callback(radar_ymdhmsz::radarprocess::Request  &req, radar_ymdhmsz::radarprocess::Response &res){

   std::stringstream c;
   c<<req.radarin.c_str();
   std::stringstream ss;

   if(c.str()=="radarpause"){
      flag_radar = false;
      ss << "OK paused";
      res.radarout = ss.str();
      ROS_ERROR("PROCESS RADAR PAUSED");
      ROS_ERROR("From Client  [%s], Server says [%s]",req.radarin.c_str(),res.radarout.c_str());}
   if(c.str()=="radarstart"){
      flag_radar = true;
      ss << "OK start";
      res.radarout = ss.str();
      ROS_ERROR("PROCESS RADAR STARTED");
      ROS_ERROR("From Client  [%s], Server says [%s]",req.radarin.c_str(),res.radarout.c_str());}
         
   return true;
}



int main(int argc, char** argv){
   ros::init (argc, argv, "radar_ymdhmsz");
   ros::NodeHandle nh_;
   
   system("exec rm -r ../dl_lrgb_ymdhmsz/Radar/*");
   
   rad_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/radar/target_list_cartesian", 10, RadarCallback);
   rad_pub_      = nh_.advertise<sensor_msgs::PointCloud2>("/radar_stamp", 2);
  
   service = nh_.advertiseService("radaraction", service_callback);
      
   ros::Rate loop_rate(40);
   while(ros::ok()){    
      ros::spinOnce();
      loop_rate.sleep();
   } 
    rad_sub_.shutdown();
    rad_pub_.shutdown();
   
    return 0;
}
