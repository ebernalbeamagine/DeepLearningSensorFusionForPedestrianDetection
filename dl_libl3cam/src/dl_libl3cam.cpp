
/******************************************************************************
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
 
////////////////////////////////////////////////////////////////////////////////////////////////// 
//---This node makes the integration between the Lidar and the RGB based on the Lidar driver.---//
//////////////////////////////////////////////////////////////////////////////////////////////////

#include <dl_libl3cam.h>

bool service_callback(dl_libl3cam::process::Request  &req, dl_libl3cam::process::Response &res){

   ROS_ERROR("SERVICE CALLBACK");
   
   std::stringstream c;
   c<<req.in.c_str();
   std::stringstream ss;
   
   if(c.str()=="stop_stream"){
      error = 0;
      ss << "OK stop_stream";
      res.out = ss.str();
      ROS_ERROR("From Client  [%s], Server says [%s]",req.in.c_str(),res.out.c_str());
      error = STOP_STREAM(m_devices[0]);
        
      if(error == L3CAM_OK){
         m_device_streaming = false;
         ROS_ERROR("STOP STREAM");
      }else{
            std::cout<<"Stop stream response  "<<error<< "  " <<getBeamErrorDescription(error);
      }
   }   
   if(c.str()=="stop_device"){
      error = 0;
      ss << "OK stop_device";
      res.out = ss.str();
      ROS_ERROR("From Client  [%s], Server says [%s]",req.in.c_str(),res.out.c_str());
      error = STOP_DEVICE(m_devices[0]);
      
      if(error == L3CAM_OK){    
         m_device_started = false;
         ROS_ERROR("STOP DEVICE");   
        }else{
            std::cout<<"Stop device response -"<<error<< " " << getBeamErrorDescription(error);
        }
   }
   return true;
}



int main (int argc, char** argv){
   ros::init (argc, argv, "dl_libl3cam");
   ros::NodeHandle nh_;
  
   m_devices_connected = 0;
   m_sensors_connected = 0;
   m_device_started    = false;
   m_device_streaming  = false;
     
   const char *version = NULL;
   version = GET_VERSION();
   printf("Library version %s \n", version);
    
    
   //!set searching status GUI
   error = 0;
   const char *desc = NULL;
   error = INITIALIZE();
    
   desc = getBeamErrorDescription(error);
   printf("Initialize response %d - %s \n", error, desc);
   
   int32_t status = 0;
   error = 0;
    
   while(m_devices_connected <1){
      std::cout << "m_devices_connected "<<  m_devices_connected <<"\n";
      FIND_DEVICES(m_devices, &m_devices_connected);
      usleep(500000);
   }
     
   error = GET_DEVICE_STATUS(m_devices[0], &status);
    
   std::cout<<"Device status "<<status<<"\n";

   error = GET_SENSORS_AVAILABLE(m_devices[0], m_sensors, &m_sensors_connected);

   std::cout<<"Found  "<<m_sensors_connected<<"  sensors in device"<<"\n";
    
    
    
    //////////////////////////////
    
   error = 0;
   
   error = START_DEVICE(m_devices[0]);
        
   if(error == L3CAM_OK){
      m_device_started = true;
      std::cout<<" The device   has started "<< "\n";
   }else{
      desc = getBeamErrorDescription(error);
      ROS_ERROR("START DEVICE Error %d - %s \n", error, desc);
            
   }
    
    
    ////////////////////////////////
    
    
    /////////////////////////////////
    
   error = 0;
   error = START_STREAM(m_devices[0]);
   if(error == L3CAM_OK){
      m_device_streaming = true;
      std::cout<<" The device   is streaming "<< "\n";
   }else{
      desc = getBeamErrorDescription(error);
      ROS_ERROR("STREAM DEVICE Error %d - %s \n", error, desc);
      m_device_streaming = false;
   }
    
    //////////////////////////////////

 
   service = nh_.advertiseService("l3cam", service_callback);
   ROS_INFO("Ready to receive from client.");
    
     
   ros::Rate loop_rate(1);
   while(ros::ok()){
      ros::spinOnce();
      loop_rate.sleep(); 
    }
}
