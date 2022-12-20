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
 
///////////////////////////////////////////////////
//---This node computes the directional vector---//
///////////////////////////////////////////////////




#include<functions.h>
#include <Eigen/Geometry>
#include<dl_err_opt_lid_rad.h>
  
 //////////////////////////////////////////
 //------Handle the Image publisher------//
 ////////////////////////////////////////// 
cv::Mat ImagePublisher(vector<std::string> file_name_listimg_,int cnti){ 
   cv::Mat image_gray(1080,1920,CV_8UC1);
   cv::Mat image_rgb(1080, 1920, 3);
   image_gray = cv::imread(file_name_listimg_[cnti], cv::IMREAD_GRAYSCALE);

   // Check if image is loaded fine
   if(image_gray.empty()){
      printf(" Error opening image\n");
   }
   
   //Check image type and resolution
   string ty =  type2str(image_gray.type() );
   
   cv_bridge::CvImage img_bridge;
   sensor_msgs::Image img_msg; // >> message to be sent
   
   std_msgs::Header header; // empty header
   ///header.seq = counter; // user defined counter
   header.stamp = ros::Time::now(); // time
   header.frame_id  =   "lidar";
   img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_8UC1, image_gray);
   img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
   img_pub_.publish(img_msg); 
    
   return image_gray;
   } 
 /////////
 //-END-//
 /////////

//////////////////////////////////////////
//------Handle the Lidar publisher------//
////////////////////////////////////////// 
void LidarPublisher(vector<std::string> file_name_listlid_,int cntl,ros::Time time){
   sensor_msgs::PointCloud2 output_li; 
   if(pcl::io::loadPCDFile<pcl::PointXYZ> (file_name_listlid_[cntl], *cloud_li) == -1){ //* load the file  
      PCL_ERROR ("Couldn't read pcd file  \n");
   }
   try{
      pcl::toROSMsg (*cloud_li, output_li); //convert the cloud
   }
   catch (std::runtime_error e){
      ROS_ERROR_STREAM("Error in converting cloud to image message: " << e.what());
   }
   output_li.header.frame_id = "lidar";
   output_li.header.stamp = time;  
       
   li_pub_.publish(output_li);  
   
}   
/////////
//-END-//
/////////

 
  
//////////////////////////////////////////
//------Handle the Radar publisher------//
////////////////////////////////////////// 
void RadarPublisher(vector<std::string> file_name_listrad_, int cntr){
   if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_name_listrad_[cntr], *cloud_ra) == -1){ //* load the file  
      PCL_ERROR ("Couldn't read pcd file  \n");
   }
   try{
      sensor_msgs::PointCloud cloud_;
      cloud_.points.resize(cloud_ra->size ());
      cloud_.header.frame_id = "radar";
      cloud_.header.stamp    = ros::Time::now();                      
 
      int kk=0;
      
      
      std::cout<<" /////////////////---Radar points-----//////////////// "  << std::endl;
      for (const auto& pointcf: *cloud_ra){
         int dr = sqrt(pow(pointcf.x, 2.0) + pow(pointcf.y, 2.0) + pow(pointcf.z, 2.0))/1;
         //std::cout << " dr " << dr<< "\n";
         cloud_.points[kk].x = pointcf.x;
         cloud_.points[kk].y = pointcf.y;
         cloud_.points[kk].z = pointcf.z;
         std::cout  <<"radar point [x,y,z]"       << "    " << cloud_.points[kk].x << " "
                                                  << cloud_.points[kk].y << " "
                                                  << cloud_.points[kk].z << std::endl;
         kk++;
                
       }
       std::cout<<" ////////////////////////////////////////////////// "  << std::endl;
       sensor_msgs::PointCloud2 output_ra;
       output_ra.header.frame_id = "radar";
       output_ra.header.stamp = ros::Time(0); 
          
       sensor_msgs::convertPointCloudToPointCloud2(cloud_,output_ra);
       ra_pub_.publish(output_ra);  
   }
   catch (std::runtime_error e){
      ROS_ERROR_STREAM("Error in converting cloud to image message: " << e.what());
   }  
}
/////////
//-END-//
/////////

int main(int argc, char **argv){  
   ros::init(argc, argv, "dl_err_opt_lid_rad");
   ros::NodeHandle nh_;
 
   rad_xyz_.clear();
   radar_transformation_.resize(4,4); 
   boost::numeric::ublas::vector<float> radar_vector_(4);
   std::vector<float> radv;
   radv.clear();  
   float number;  
   
   std::ifstream ifs ("./radar_matrix.txt");
 
   if(ifs.is_open()){
      while(ifs >> number){  
         radv.push_back(number);
      }
  }
  else {
     std::cout << "Error opening file";
  }

   for( size_t i = 0; i < radv.size()/3; i++ ){ 
      RAD_COORDINATE s_xyz_;
      s_xyz_.x    = radv[3*i+0];
      s_xyz_.y    = radv[3*i+1];
      s_xyz_.z    = radv[3*i+2];
      
      rad_xyz_.push_back(s_xyz_); 
   }
 
   radar_transformation_(0,0) =  0.99855536;
   radar_transformation_(0,1) = -0.01588738; 
   radar_transformation_(0,2) =  0.05133007;
   radar_transformation_(0,3) =  0.00000000;
   radar_transformation_(1,0) =  0.00908447;
   radar_transformation_(1,1) =  0.9914536;
   radar_transformation_(1,2) =  0.13014313;
   radar_transformation_(1,3) =  0.00000000;
   radar_transformation_(2,0) =  0.05295902;
   radar_transformation_(2,1) =  0.12948881;
   radar_transformation_(2,2) = -0.99016564;
   radar_transformation_(2,3) =  0.00000000;
   radar_transformation_(3,0) =  0.0880444;
   radar_transformation_(3,1) =  0.09847893;
   radar_transformation_(3,2) = -0.02436598;
   radar_transformation_(3,3) =  1.00000;

   
   nh_.param("/gray16", gray16, false);
   nh_.param("/gray8" , gray8 , false);
   
   int cntl = 0;
   int cntr = 0;
   int cnti = 0;
  
  
   //! TODO: Specify the path for the images and get all  the RGB images 
   std::string pathi_ = "./lidar_radar_rgb_data/RGB/";
   vector<std::string>   file_name_listimg_ = GetAllImagesFromFolder(pathi_);
    
   //! TODO: Specify the path for the Lidar and get all  the Lidar images
   std::string pathl_ = "./lidar_radar_rgb_data/Lidar/";
   vector<std::string>   file_name_listlid_= GetAllLidarPointCloudFromFolder(pathl_);
    
    
    //! TODO: Specify the path for the Radar and get all  the Radar images
   std::string pathr_ = "./lidar_radar_rgb_data/Radar/";
   vector<std::string>   file_name_listrad_ = GetAllRadarPointCloudFromFolder(pathr_);
    
    
   int size =file_name_listlid_.size();
   
   chatter_pub      =  nh_.advertise<std_msgs::String>("chatter", 10);
   li_pub_          =  nh_.advertise<sensor_msgs::PointCloud2>("/lidar_calibration", 2); 
   ra_pub_          =  nh_.advertise<sensor_msgs::PointCloud2>("/radar_calibration", 2); 
   im_pub_          =  nh_.advertise<sensor_msgs::PointCloud2>("/image_calibration", 2);
   img_pub_         =  nh_.advertise<sensor_msgs::Image>("/image", 2);
   dirv_pub_        =  nh_.advertise<geometry_msgs::Vector3Stamped>("/dirvector", 10);
   radarcenter_pub_ =  nh_.advertise<visualization_msgs::Marker>( "/radar_center", 2 );

   ros::Rate loop_rate(1);

  
   int count = 1;
   while (ros::ok()){
      getchar();
      ros::Time time = ros::Time::now();
     
      cv::Mat image_gray = ImagePublisher(file_name_listimg_,cnti);
      cnti++;
 
      //Calculate the Image Center of the calibration board  
      auto[x, y] = ImageCenterBoardPosition(image_gray);
      
      std::cout<<" ////////////////////////////////////////////////// " << std::endl;
      std::cout<<" /////------image ----" << count<<"     ///////////"<<  std::endl;
      std::cout<<" ////////////////////////////////////////////////// " << "\n" << std::endl;
      
      //Publish the Lidar cloud
      LidarPublisher(file_name_listlid_,cntl,time);
  
      //Publish the Radar cloud
      RadarPublisher(file_name_listrad_,cntr);
      cntr++;
    
     
      //Get the directional vector  
      auto[cxl, cyl, czl] = DirectionalVector(file_name_listlid_,time,cntl,x,y,gray16,gray8);
      cntl++;
    
      
      //Send the directional vector
      geometry_msgs::Vector3Stamped dirvector;;
      dirvector.header.stamp = time;
      dirvector.vector.x = cxl;
      dirvector.vector.y = cyl;
      dirvector.vector.z = czl;
      dirv_pub_.publish(dirvector);
  
   
      /////////////////////////////////////////////////////////////
      //------Handle the lidar center position of the board------//
      ///////////////////////////////////////////////////////////// 
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  
      lidar_posx_.clear();
      lidar_posy_.clear();
      lidar_posz_.clear();
      kl=0;
      //Generate the data
      for (auto& pointl: *cloud_li){
         lidar_posx_.push_back(pointl.x);
         lidar_posy_.push_back(pointl.y);
         lidar_posz_.push_back(pointl.z);
         kl++;
      }
      /////////
      //-END-//
      /////////
   
   
      /////////////////////////////////////////////////////////////
      //------Handle the radar center position of the board------//
      /////////////////////////////////////////////////////////////  
      radar_posx_.clear();
      radar_posy_.clear();
      radar_posz_.clear();
      kr=0;
      //Generate the data
      for(auto& pointr: *cloud_ra){
         radar_posx_.push_back(pointr.x);
         radar_posy_.push_back(pointr.y);
         radar_posz_.push_back(pointr.z);
         kr++;
      }
  
      int radar_size = radar_posx_.size();
      /////////
      //-END-//
      /////////
 
      std::cout << rad_xyz_[count-1].x<< " \n";
      std::cout << rad_xyz_[count-1].y<< " \n";
      std::cout << rad_xyz_[count-1].z<< " \n";
   
      radar_vector_(0) = rad_xyz_[count-1].x;
      radar_vector_(1) = rad_xyz_[count-1].y;
      radar_vector_(2) = rad_xyz_[count-1].z;
      radar_vector_(3) = 1.0; 
    
      radar_point = boost::numeric::ublas::prec_prod(radar_vector_, radar_transformation_); 
      std::cout  <<"radar center point [x,y,z]"       << "    " << radar_point(0) << " "
                                                             << radar_point(1) << " "
                                                             << radar_point(2) << " "
                                                             << radar_point(3) << std::endl;
      radarc_marker.header.frame_id = "lidar";
      radarc_marker.header.stamp = ros::Time();
      radarc_marker.ns = "radar_center_of_interest";
      radarc_marker.id = 0;
      radarc_marker.type = visualization_msgs::Marker::SPHERE;
      radarc_marker.action = visualization_msgs::Marker::ADD;
      radarc_marker.pose.position.x =radar_point(0)/radar_point(3);
      radarc_marker.pose.position.y =radar_point(1)/radar_point(3);
      radarc_marker.pose.position.z =radar_point(2)/radar_point(3);
      radarc_marker.pose.orientation.x = 0.0;
      radarc_marker.pose.orientation.y = 0.0;
      radarc_marker.pose.orientation.z = 0.0;
      radarc_marker.pose.orientation.w = 1.0;
      radarc_marker.scale.x = 0.13;
      radarc_marker.scale.y = 0.13;
      radarc_marker.scale.z = 0.13;
      radarc_marker.color.a = 1.0; // Don't forget to set the alpha!
      radarc_marker.color.r = 1.0;
      radarc_marker.color.g = 0.0;
      radarc_marker.color.b = 0.0;	
      
      radarcenter_pub_.publish(radarc_marker);
  
      if(cntr==size){
         std::cout<< "\n" << std::endl;
         std::cout<<" ////////////////////////////////////////////////// " << "\n" << std::endl;
         std::cout<<" ////----The Node has finished :-)  :-) ----/////// " << "\n" << std::endl;
         std::cout<<" ////////////////////////////////////////////////// " << "\n" << std::endl;
         count=0;
         cntr=0;
         cntl=0;
         cnti=0;
      }
      count++;
   
      ros::spinOnce();
      loop_rate.sleep();
   }


   li_pub_.shutdown();
   ra_pub_.shutdown();
   im_pub_.shutdown();
   PC2_pub_.shutdown();
   img_pub_.shutdown();
   dirv_pub_.shutdown();
   radarcenter_pub_.shutdown();
   return 0;
}
