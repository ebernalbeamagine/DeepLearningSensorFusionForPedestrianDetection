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
 
//////////////////////////////////////////////////////////////////////////// 
//---This node receives the socket data and publish a lidar pointcloud.---//
////////////////////////////////////////////////////////////////////////////

#include<dl_lidarpointcloud.h>

void error(const char *msg){
   perror(msg);
}

void *PointCloudThread(void *functionData){
   struct sockaddr_in m_socket;  
   int m_socket_descriptor;             //! Socket descriptor 
   std::string m_address ="192.168.1.23";  //"127.0.0.1";//"192.168.1.110";//"10.192.215.173";// "192.168.1.25"; //! Local address of the network interface port connected to the L3CAM 
   int m_udp_port =  6050; //!For the  pointcloud is 6050 
   
   socklen_t socket_len = sizeof(m_socket);
   char* buffer;
   buffer = (char*)malloc(64000);
  
   int32_t    m_pointcloud_size;
   int32_t   *m_pointcloud_data;
   uint32_t   m_timestamp;
   bool       m_is_reading_pointcloud;
   int        points_received = 1;
   int        k=0;
   int        m=0;
   
    
   if((m_socket_descriptor = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1){
      error("Opening socket");
      return 0;
   }
   else{std::cout<<"Socket dl_lidarpointcloud created"<<"\n";}
      memset((char *) &m_socket, 0, sizeof(struct sockaddr_in));  
      m_socket.sin_addr.s_addr = inet_addr ((char*)m_address.c_str());
      m_socket.sin_family = AF_INET;
      m_socket.sin_port = htons(m_udp_port);
     
   if(inet_aton((char*)m_address.c_str(), &m_socket.sin_addr) == 0){
      error("inet_aton() failed");
      return 0;
   }
   if(bind(m_socket_descriptor, (struct sockaddr *)&m_socket, sizeof(struct sockaddr_in)) == -1){
      error("Could not bind name to socket");
      close(m_socket_descriptor);
      return 0;
   }
    
   int rcvbufsize = 134217728;

   if(0 != setsockopt(m_socket_descriptor,SOL_SOCKET,SO_RCVBUF,(char*)&rcvbufsize,sizeof(rcvbufsize))){
      error("Error setting size to socket");
      return 0;
   } 
    

   while(true){
      int size_read = recvfrom(m_socket_descriptor, buffer, 64004, 0, (struct sockaddr*)&m_socket, &socket_len);
      if(size_read == 17){
         memcpy(&m_pointcloud_size, &buffer[1], 4);
         m_pointcloud_data = (int32_t*)malloc(sizeof(int32_t)*(((m_pointcloud_size) *5) + 1));
         memcpy(&m_pointcloud_data[0], &m_pointcloud_size, sizeof(int32_t));
         int32_t suma_1, suma_2;
         memcpy(&suma_1, &buffer[5], sizeof(int32_t));
         memcpy(&suma_2, &buffer[9], sizeof(int32_t));
         memcpy(&m_timestamp, &buffer[13], sizeof(uint32_t));
         m_is_reading_pointcloud = true;
         points_received = 1;
      }
      else if(size_read == 1){
         m_is_reading_pointcloud = false;
         int32_t *data_received = (int32_t*)malloc(sizeof(int32_t)*(m_pointcloud_size*5)+1);
         memcpy(&data_received[0], &m_pointcloud_data[0],sizeof(int32_t)*((m_pointcloud_size*5)+1)); 
         int size_pc = data_received[0];
         sensor_msgs::PointCloud cloud_;
         cloud_.points.resize(size_pc);
         cloud_.header.frame_id = "lidar";
         cloud_.header.stamp    = ros::Time::now(); 
          
         for(int i = 0; i < size_pc; i++){
            cloud_.points[i].y     =   -(double)data_received[5*i+1]/1000; 
            cloud_.points[i].z     =   -(double)data_received[5*i+2]/1000; 
            cloud_.points[i].x     =    (double)data_received[5*i+3]/1000;     
         }
          
         sensor_msgs::PointCloud2 PC2_msg; 
         PC2_msg.header.frame_id = "lidar";
         PC2_msg.header.stamp = ros::Time(0); 
          
         sensor_msgs::convertPointCloudToPointCloud2(cloud_,PC2_msg);
         PC2_pub_.publish(PC2_msg);
         free(m_pointcloud_data);
         points_received = 1;
        }
        else if(size_read > 0){
           if(m_is_reading_pointcloud){
              int32_t points = 0;
              memcpy(&points, &buffer[0], 4);
              memcpy(&m_pointcloud_data[points_received],&buffer[4],
              (sizeof(int32_t)*(points*5)));
              points_received+=points*5;
           }
        }
   }
   free(buffer); 
   return 0;
}

int main (int argc, char** argv){
   ros::init (argc, argv, "dl_lidarpointcloud");
   ros::NodeHandle nh_;
   
   pthread_create (&pointcloud_thread, NULL, PointCloudThread, NULL); 
     
   PC2_pub_     = nh_.advertise<sensor_msgs::PointCloud2>("/PC2_lidar", 2);
     
   ros::Rate loop_rate(1);
   while(ros::ok()){
       ros::spinOnce();
       loop_rate.sleep();   
   }
  
    PC2_pub_.shutdown();
}
