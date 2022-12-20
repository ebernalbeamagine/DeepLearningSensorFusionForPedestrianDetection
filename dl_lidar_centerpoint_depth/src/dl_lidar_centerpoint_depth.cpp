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
 
/////////////////////////////////////////////////////////////
//---This node computes the lidar 3D point x_l, y_l, z_l---//
/////////////////////////////////////////////////////////////

#include <dl_lidar_centerpoint_depth.h>




//Callback function that makes sensor data readings alignment.
void callback(const  sensor_msgs::PointCloud2ConstPtr& pc2_msg_, const geometry_msgs::Vector3StampedConstPtr& dirv_msg_){
   ///////////////////////////////////////////////
   /////----Region of Interest---/////////////////
   ///////////////////////////////////////////////
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloudlidar(new pcl::PointCloud<pcl::PointXYZ>);
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_model (new pcl::PointCloud<pcl::PointXYZ>);

   posx_.clear();
   posy_.clear();
   posz_.clear();

   k=0;
   for (sensor_msgs::PointCloud2ConstIterator<float> it(*pc2_msg_, "x"); it != it.end(); ++it) {
      // TODO: do something with the values of x, y, z
      //std::cout << it[0] << ", " << it[1] << ", " << it[2] << ", " << it[3] <<"\n \n";
      posx_.push_back(it[0]);
      posy_.push_back(it[1]);
      posz_.push_back(it[2]);
   }
    
   // Fill in the cloud data
   cloudlidar->width  = pc2_msg_->width;
   cloudlidar->height = 1;
   cloudlidar->points.resize (cloudlidar->width * cloudlidar->height);
    
   // Generate the data
   for (auto& point: *cloudlidar){
      point.x = posx_[k];
      point.y = posy_[k];
      point.z = posz_[k];
      k++;
   }
 
   // Create the filtering object
   pcl::PassThrough<pcl::PointXYZ> pass;
   pass.setInputCloud (cloudlidar);

   pass.setFilterFieldName ("y");
   //pass.setFilterLimits (-2.5, 1.5);
   pass.setFilterLimits (-1.5, -0.2);
   //pass.setFilterLimitsNegative (true);
 
   if(cnnt<9){
      pass.setFilterFieldName ("x");
      pass.setFilterLimits (3.0, 4.5);    
   }
   
   
   if(cnnt==11){
      pass.setFilterFieldName ("x");
      pass.setFilterLimits (13.5, 14.0); 
   }
   cnnt++;
   pass.filter (*cloud_filtered);
                         
   sensor_msgs::PointCloud cloud_;
   cloud_.points.resize(cloud_filtered->size ());
   cloud_.header.frame_id = "lidar";
   cloud_.header.stamp    = ros::Time::now(); 
   
   
   int kk=0;
   for (const auto& pointcf: *cloud_filtered){
      cloud_.points[kk].x = pointcf.x;
      cloud_.points[kk].y = pointcf.y;
      cloud_.points[kk].z = pointcf.z;
      kk++;
   }
   
   sensor_msgs::PointCloud2 pc2_; 
   pc2_.header.frame_id = "lidar";
   pc2_.header.stamp = ros::Time(0); 
          
   sensor_msgs::convertPointCloudToPointCloud2(cloud_,pc2_);

   PC2_pub_.publish(pc2_);
   ///////////
   //--END--//
   ///////////



   /////////////////////////////////////////
   ////--Compute the RoI model [A,B,C,D]--///
   /////////////////////////////////////////
   posmx_.clear();
   posmy_.clear();
   posmz_.clear();
   for (sensor_msgs::PointCloud2ConstIterator<float> it(pc2_, "x"); it != it.end(); ++it) {
      // TODO: do something with the values of x, y, z
      //std::cout << it[0] << ", " << it[1] << ", " << it[2] << ", " << it[3] <<"\n \n";
      posmx_.push_back(it[0]);
      posmy_.push_back(it[1]);
      posmz_.push_back(it[2]);
   }

   // Fill in the cloud data
   cloud_model->width  = pc2_.width;
   cloud_model->height = 1;
   cloud_model->points.resize (cloud_model->width * cloud_model->height);
   int km=0;
   
   // Generate the data
   for (auto& point: *cloud_model){
      point.x = posmx_[km];
      point.y = posmy_[km];
      point.z = posmz_[km];
      km++;
      //std::cout << "point.x" << point.x << "\n";
   }
   
   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
   pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
   // Create the segmentation object
   pcl::SACSegmentation<pcl::PointXYZ> seg;	
   // Optional
   seg.setOptimizeCoefficients (true);
   // Mandatory
   seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
   seg.setMethodType (pcl::SAC_RANSAC);
   seg.setDistanceThreshold (0.2);
  
  
   seg.setInputCloud (cloud_model);
   seg.segment (*inliers, *coefficients);
   //std::cerr << "Model2 inliers: " << inliers->indices.size () << std::endl;

   if (inliers->indices.size () == 0){
      PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
   }

   //std::cerr << "Model coefficients: " << "A: " << coefficients->values[0] << " " 
   //                                    << "B: " << coefficients->values[1] << " "
   //                                    << "C: " << coefficients->values[2] << " " 
   //                                    << "D: " << coefficients->values[3] << std::endl;

  
   float A = coefficients->values[0];
   float B = coefficients->values[1];
   float C = coefficients->values[2];
   float D = coefficients->values[3];
  
   sensor_msgs::PointCloud cloudp_;
   cloudp_.points.resize(inliers->indices.size ());
   cloudp_.header.frame_id = "lidar";
   cloudp_.header.stamp    = ros::Time::now(); 
   
   
   int kkl=0;
   //for (int idx = 0; idx  < inliers->indices.size();  idx++){
   for (const auto& idx: inliers->indices){
      cloudp_.points[kkl].x = cloud_model->points[idx].x;
      cloudp_.points[kkl].y = cloud_model->points[idx].y;
      cloudp_.points[kkl].z = cloud_model->points[idx].z;
      kkl++;
   }


   sensor_msgs::PointCloud2 pc2m_; 
   pc2m_.header.frame_id = "lidar";
   pc2m_.header.stamp = ros::Time(0); 
          
   sensor_msgs::convertPointCloudToPointCloud2(cloudp_,pc2m_);

   PC2m_pub_.publish(pc2m_);
   
   ///////////
   //--END--//
   ///////////



   ///////////////////////////////////////
   /////---Compute the intersection---////  
   ///////////////////////////////////////  

   ////////////////////////////////////////////////////////////////////////////////
   //define the vector n which is a perpendicular vector to the plane model
   //[A,B,C,D]  
   ////////////////////////////////////////////////////////////////////////////////
   boost::numeric::ublas::vector<float> n_(3);
   n_(0) = A;
   n_(1) = B;
   n_(2) = C;
   
   
   //////////////////////////////////////////////////////////
   //pm = (x,y,z) point on the plane from the  pointcloud  
   //x=-(B*y+C*z+D)/A
   /////////////////////////////////////////////////////////  
   boost::numeric::ublas::vector<float> vo_(3);
   int SIZEM = inliers->indices.size ();
   float pmy = cloud_model->points[int(SIZEM/2)].y;
   float pmz = cloud_model->points[int(SIZEM/2)].z;
   float pmx = -(B*pmy+C*pmz+D)/A;
   vo_(0) =  pmx;
   vo_(1) =  pmy;
   vo_(2) =  pmz;

   /////////////////////////////////////////////////////////////////////////////////////////////////////
   //Initial point of the line which comes from the parameters in the fusion_matrix.txt
   //ECON_LOCATION_FROM_DASA_X (camera_location_from_lidar_x)
   //[-2.018464	-2.731670 102.328893] in lidar coordinates [x,y,z] it is changed to ROS coordinates
   //and converted to meters, [z,-x,-y] = [0.102328893, 0.002018464, 0.002731670]  
   //////////////////////////////////////////////////////////////////////////////////////////////////////
   boost::numeric::ublas::vector<float> po_(3);
   po_(0) = 0.1023;
   po_(1) = 0.002018464;
   po_(2) = 0.002731670;

   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   //The line vector from the center of the camera seeing by the Lidar frame, this vector has been computed by the 
   //ROS node err_opt_lid_rad
   /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   boost::numeric::ublas::vector<float> u_(3);
   float vy = -dirv_msg_->vector.x;
   float vz = -dirv_msg_->vector.y;
   float vx =  dirv_msg_->vector.z;
   std::cout<<"\n  vx "<<dirv_msg_->vector.x<<"\n";
   std::cout<<"\n  vy "<<dirv_msg_->vector.y<<"\n";
   std::cout<<"\n  vz "<<dirv_msg_->vector.z<<"\n";
   u_(0) =  vx;
   u_(1) =  vy;
   u_(2) =  vz;
   
   ///////////////////////////////
   //Initial point
   //////////////////////////////
   
   //pointp_(0) = 0.0000;
   //pointp_(1) = 0.0000;
   //pointp_(2) = 0.0000;

   
   boost::numeric::ublas::vector<float> w_(3);
   w_(0) = po_(0)-vo_(0);
   w_(1) = po_(1)-vo_(1);
   w_(2) = po_(2)-vo_(2);

   ////////////////////////////////////
   //Dot product between n and u
   ///////////////////////////////////
   float D_;
   D_ = (n_(0)*u_(0))+(n_(1)*u_(1))+(n_(2)*u_(2));
   //std::cout << "\n " << "D  " << D_ << " \n";

   ////////////////////////////////////
   //Dot product between n and w
   ///////////////////////////////////
   float N_;
   N_ = -((n_(0)*w_(0))+(n_(1)*w_(1))+(n_(2)*w_(2)));
   //std::cout << "\n " << "N  " << N_ << " \n";

   //////////////////////////////////////////////////////////////////////////////////////
   //Compute the depth vector from the center of the camera seeing by the Lidar frame 
   //to the center point of the calibration board
   //////////////////////////////////////////////////////////////////////////////////////
   float sl_ = N_ / D_;
   //std::cout << "\n " << "sl  " << sl_ << " \n";
   pointp_ = po_+ (sl_*u_);
   
   
   std::cout<< "\n" << std::endl;
   
   std::cout<<" ////////////////////////////////////////////////// "  << std::endl;
   std::cout<<" ////---- This is the depth vector [x,y,z] ----//// "  << std::endl; 
   std::cout<<" ////////////////////////////////////////////////// "  << std::endl;
   std::cout  <<"lidar depth point [x,y,z]"       << "    " << pointp_(0) << " "
                                                       << pointp_(1) << " "
                                                       << pointp_(2) << std::endl;
   std::cout<<" ////////////////////////////////////////////////// "  << std::endl;                                                   



  

///////////
//--END--//
///////////
   
}

int main (int argc, char** argv){
   ros::init (argc, argv, "dl_lidar_centerpoint_depth");
   ros::NodeHandle nh_;
 
   message_filters::Subscriber<sensor_msgs::PointCloud2> laser_sub_(nh_, "/lidar_calibration", 10);
   message_filters::Subscriber<geometry_msgs::Vector3Stamped> dirv_sub_(nh_,  "/dirvector", 10);
 
   typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2,geometry_msgs::Vector3Stamped> MySyncPolicy;
   Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),  laser_sub_, dirv_sub_);
   sync.registerCallback(boost::bind(&callback, _1, _2));
    
    
   PC2_pub_         =  nh_.advertise<sensor_msgs::PointCloud2>("/filtered", 2);
   PC2m_pub_        =  nh_.advertise<sensor_msgs::PointCloud2>("/model_lidar", 2);
   lidarcenter_pub_ =  nh_.advertise<visualization_msgs::Marker>( "/lidar_center", 2 );
    
   ros::Rate loop_rate(10);
   while(ros::ok()){
    
      lidarc_marker.header.frame_id = "lidar";
      lidarc_marker.header.stamp = ros::Time();
      lidarc_marker.ns = "lidar_center_of_interest";
      lidarc_marker.id = 0;
      lidarc_marker.type = visualization_msgs::Marker::SPHERE;
      lidarc_marker.action = visualization_msgs::Marker::ADD;
      lidarc_marker.pose.position.x =pointp_(0);
      lidarc_marker.pose.position.y =pointp_(1);
      lidarc_marker.pose.position.z =pointp_(2);
      lidarc_marker.pose.orientation.x = 0.0;
      lidarc_marker.pose.orientation.y = 0.0;
      lidarc_marker.pose.orientation.z = 0.0;
      lidarc_marker.pose.orientation.w = 1.0;
      lidarc_marker.scale.x = 0.1;
      lidarc_marker.scale.y = 0.1;
      lidarc_marker.scale.z = 0.1;
      lidarc_marker.color.a = 1.0; // Don't forget to set the alpha!
      lidarc_marker.color.r = 0.0;
      lidarc_marker.color.g = 0.0;
      lidarc_marker.color.b = 1.0;	
      ros::spinOnce();
      loop_rate.sleep();
      lidarcenter_pub_.publish( lidarc_marker);
   }

   
   // clean up subscribers and publishers 
   PC2_pub_.shutdown();
   PC2m_pub_.shutdown();
   lidarcenter_pub_.shutdown();
   return 0; 
}
