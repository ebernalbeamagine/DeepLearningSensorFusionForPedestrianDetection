/*************************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C)  
 *
 * This file is part of software developed by UPC-CD6-Beamagin group.
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
 
//////////////////////////////////////////////////////////// 
//---This node service pauses and starts the image node---//
////////////////////////////////////////////////////////////

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>
#include "dl_image_ymdhmsz/imgprocess.h"	
#include <iostream>
#include <sstream>

using namespace std;


int main(int argc, char **argv){
   ros::init(argc, argv, "image_start_pause_client");

   ros::NodeHandle n;
   ros::Rate loop_rate(10);

   ros::ServiceClient imgclient = n.serviceClient<dl_image_ymdhmsz::imgprocess>("imgaction");
   dl_image_ymdhmsz::imgprocess srv;
   std::stringstream imgss;
   imgss << "imgstart";
   srv.request.imgin = imgss.str();

   if (imgclient.call(srv)){
     ROS_INFO("From Client  [%s], Server says [%s]",srv.request.imgin.c_str(),srv.response.imgout.c_str());
   }
   else{
      ROS_ERROR("Failed to call service");
      return 1;
   }

   while (ros::ok()){
      ros::spinOnce();
      loop_rate.sleep();
   }
   return 0;
}
