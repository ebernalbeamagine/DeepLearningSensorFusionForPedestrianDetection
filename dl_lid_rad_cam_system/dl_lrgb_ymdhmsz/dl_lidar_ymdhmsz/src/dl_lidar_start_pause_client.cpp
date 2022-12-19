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
 
//////////////////////////////////////////////////////////// 
//---This node service pauses and starts the lidar node---//
////////////////////////////////////////////////////////////


#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>
#include "dl_lidar_ymdhmsz/lidarprocess.h"	
#include <iostream>
#include <sstream>





using namespace std;

int main(int argc, char **argv){
   ros::init(argc, argv, "lidar_start_pause_client");
   ros::NodeHandle n;
   ros::Rate loop_rate(10);

   ros::ServiceClient lidarclient = n.serviceClient<dl_lidar_ymdhmsz::lidarprocess>("lidaraction");
   dl_lidar_ymdhmsz::lidarprocess srv;
   std::stringstream lidarss;
   lidarss << "lidarstart";
   srv.request.lidarin = lidarss.str();

   if (lidarclient.call(srv)){
      ROS_INFO("From Client  [%s], Server says [%s]",srv.request.lidarin.c_str(),srv.response.lidarout.c_str());
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
