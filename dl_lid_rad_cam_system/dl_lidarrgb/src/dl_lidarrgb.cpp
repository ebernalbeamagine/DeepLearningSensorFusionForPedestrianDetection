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
 
///////////////////////////////////////////////////////////////////// 
//---This node receives the socket data and publish a rgb image.---//
/////////////////////////////////////////////////////////////////////

#include<dl_lidarrgb.h>

void *ImageThread(void *functionData){
   struct sockaddr_in m_socket;  
   int m_socket_descriptor;             //! Socket descriptor 
   std::string m_address = "192.168.1.23";//"127.0.0.1";//"192.168.1.110";//"10.192.215.173";// "192.168.1.25"; //! Local address of the network interface port connected to the L3CAM 
   int m_udp_port =  6020; //!PARA LA RGB ES 6020 
   socklen_t socket_len = sizeof(m_socket);
   char* buffer;
   buffer = (char*)malloc(64000);
 
   uint16_t  m_image_height;
   uint16_t  m_image_width;
   uint8_t   m_image_channels;
   uint32_t  m_timestamp;
   uint8_t   m_image_detections;
   int       m_image_data_size;
   bool      m_is_reading_image;
   bool      m_image_ready;
   char     *m_image_buffer;
   int       bytes_count = 0;
   int       b_count = 0;
   detectionImage curr_det;
  
  
  std::vector<detectionImage> m_detections;

   if((m_socket_descriptor = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1){
      error("Opening socket");
      return 0;
   }
   else{std::cout<<"Socket dl_lidarrgb created"<<"\n";}
      memset((char *) &m_socket, 0, sizeof(struct sockaddr_in));  
      m_socket.sin_addr.s_addr = inet_addr ((char*)m_address.c_str());
      m_socket.sin_family = AF_INET;
      m_socket.sin_port = htons(m_udp_port);
   if (inet_aton((char*)m_address.c_str(), &m_socket.sin_addr) == 0){
      error("inet_aton() failed");
      return 0;
   }
   if (bind(m_socket_descriptor, (struct sockaddr *)&m_socket, sizeof(struct sockaddr_in)) == -1){
      error("Could not bind name to socket");
      close(m_socket_descriptor);
      return 0;
   }
 
    int rcvbufsize = 134217728;

   if(0 != setsockopt(m_socket_descriptor,SOL_SOCKET,SO_RCVBUF,(char*)&rcvbufsize,sizeof(rcvbufsize))){
      error("Error setting size to socket");
      return 0;
   }

   int count=0;

   while(true){
      int size_read = recvfrom(m_socket_descriptor, buffer, 64004, 0, (struct sockaddr*)&m_socket, &socket_len);
      if(size_read == 11){
         memcpy(&m_image_height, &buffer[1], 2);
         memcpy(&m_image_width, &buffer[3], 2);
         memcpy(&m_image_channels, &buffer[5], 1);
         memcpy(&m_timestamp, &buffer[6], sizeof(uint32_t));
         memcpy(&m_image_detections, &buffer[10], 1);
         m_image_data_size = m_image_height*m_image_width*m_image_channels;
         m_image_buffer = (char*)malloc(m_image_data_size);
         m_is_reading_image = true;
         m_image_ready = false;
         m_detections.clear();
         bytes_count = 0; 
      }
      else if(size_read == 1 || bytes_count == m_image_data_size ){
         m_is_reading_image = false;
         m_image_ready = true;
         bytes_count = 0;
         uint8_t* image_pointer = (uint8_t*)malloc(m_image_data_size);
         memcpy(image_pointer, m_image_buffer, m_image_data_size); 
         cv::Mat img_data(m_image_height,m_image_width,CV_8UC3);
         memcpy(img_data.data, (char *) m_image_buffer, m_image_data_size);
          
         cv_bridge::CvImage img_bridge;
         sensor_msgs::Image img_msg; // >> message to be sent

         std_msgs::Header header; // empty header
         header.stamp = ros::Time::now(); // time
         header.frame_id  =   "lidar";
         img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img_data);
         img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
         img_pub_.publish(img_msg); // ros::Publisher pub_img = node.advertise<sensor_msgs::Image>("topic", queuesize);
 
          
          
          /////////////////////////////////////////////////////
          //emit imageRgbReadyToShow(image_pointer, m_image_height, m_image_width, m_image_channels, m_detections);
          free(m_image_buffer);
          m_image_detections = 0;
       }else if(size_read != -1){
           if(m_image_detections > 0){
           //   std::cout << "Here 3 "<<"\n";
          //!read detections packages
              memcpy(&curr_det.confidence, &buffer[0], 2);
              memcpy(&curr_det.box.x, &buffer[2], 2);
              memcpy(&curr_det.box.y, &buffer[4], 2);
              memcpy(&curr_det.box.height, &buffer[6], 2);
              memcpy(&curr_det.box.width, &buffer[8], 2);
              m_detections.push_back(curr_det);
              --m_image_detections;
              continue;
           }
           if(m_is_reading_image){
             // std::cout <<"\n"<< "Here 4 "<<"\n";
              memcpy(&m_image_buffer[bytes_count], buffer, size_read);
              bytes_count+=size_read;
           }
      }

   }
   free(buffer);
   return 0;
}



int main (int argc, char** argv){
   ros::init (argc, argv, "dl_lidarrgb");
   ros::NodeHandle nh_;
 
   pthread_create (&image_thread_handler, NULL, &ImageThread, NULL);
     
   img_pub_   = nh_.advertise<sensor_msgs::Image>("/img_rgb_", 2);
   
   ros::Rate loop_rate(1);
   while(ros::ok()){   
       ros::spinOnce();
       loop_rate.sleep();
    
   }
   img_pub_.shutdown();
}
