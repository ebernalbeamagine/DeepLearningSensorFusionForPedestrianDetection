/*************************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C)  
 *
 * This file is part of software developed by UPC-Cd6-Beamagin group.
 *
 * Author: Alfredo Cháez Plascencia  alfredo.chavez@upc.edu
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
 
///////////////////////////////////////////////////////////////////////////////////////
//---This node receives the RGB image  and save it as the format yyMMdd-hhmmss-zz.---//
///////////////////////////////////////////////////////////////////////////////////////


#include <dl_image_ymdhmsz.h>

void  ImageCallback(const  sensor_msgs::ImageConstPtr& img_msg_){ 
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
   
   cv_bridge::CvImagePtr cv_img_;
   try
   {
      cv_img_ = cv_bridge::toCvCopy(img_msg_, sensor_msgs::image_encodings::BGR8);
   }
   catch (cv_bridge::Exception& e)
   {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
   }
   
   char md [50];
   sprintf (md, "%02d%02d", month,day);
  
   char hms [50];
   sprintf (hms, "%02d%02d%02d", hour,min,sec);
   
   char ms [50];
   sprintf (ms, "%03d", msec);
    
   if(flag_rgb==true){    
      cv:imwrite("/home/acp/catkin_ws/src/dl_pedestrian_detection/dl_lrgb_ymdhmsz/RGB/" +   std::to_string(year)+ md + "_" + hms + "_" + ms + ".png", cv_img_->image);
  
   }
   
    img_pub_.publish(img_msg_);
}

bool service_callback(image_ymdhmsz::imgprocess::Request  &req, image_ymdhmsz::imgprocess::Response &res){

   std::stringstream c;
   c<<req.imgin.c_str();
   std::stringstream ss;

   if(c.str()=="imgpause"){
      flag_rgb   = false;
      ss << "OK paused";
      res.imgout = ss.str();
      ROS_ERROR("PROCESS RGB PAUSED");
      ROS_ERROR("From Client  [%s], Server says [%s]",req.imgin.c_str(),res.imgout.c_str());}
   if(c.str()=="imgstart"){
      flag_rgb   = true;
      ss << "OK start";
      res.imgout = ss.str();
      ROS_ERROR("PROCESS RGB STARTED");
      ROS_ERROR("From Client  [%s], Server says [%s]",req.imgin.c_str(),res.imgout.c_str());}
  
   return true;
}



int main(int argc, char** argv){
   ros::init (argc, argv, "image_ymdhmsz");
   ros::NodeHandle nh_;
  
   system("exec rm -r /home/acp/catkin_ws/src/dl_pedestrian_detection/dl_lrgb_ymdhmsz/RGB/*");
   
   img_sub_ = nh_.subscribe<sensor_msgs::Image>("/img_rgb_", 10, ImageCallback);
   img_pub_ = nh_.advertise<sensor_msgs::Image>("/img_stamp", 2);
    
   service = nh_.advertiseService("imgaction", service_callback);
      
   ros::Rate loop_rate(40);
   while(ros::ok()){
      ros::spinOnce();
      loop_rate.sleep();
   }
    
   img_sub_.shutdown();
   img_pub_.shutdown();
   return 0;
}
