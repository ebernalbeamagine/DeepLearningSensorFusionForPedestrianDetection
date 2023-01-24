# Deep Learning Sensor Fusion for Pedestrian Detection

This repository presents a fully convolutional neural network architecture for multimodal
sensor fusion, such as lidar, sonar, and RGB camera. The core of the network is based on SegNet,
a pixel-wise semantic segmentation network. A single SegNet is assigned to each sensor reading,
and the outputs are then applied to a fully connected neural network to fuse the three modalities
of sensors. Afterwards, an up-sampling network is applied to recover the fused data.  This repository  also presents an extrinsic calibration matrix method for sensor alignment.

## Package description:

* **dl_lid_rad_cam_system** This folder contains three packages and one folder with three packages. It communicates with the L3CAM lidar-sensor and save the data into a local host:

    * **dl_libl3cam** This package is a ROS-wraper for the L3CAM lidar-RGB sensor from the company Beamagine.
    
    * **dl_lidarpointcloud** This node receives a L3CAM socket data and publishes a lidar pointcloud.
    
    * **dl_lidarrgb** This package receives a L3CAM socket data and publishes a rgb image topic.
    
    * **dl_lrgb_ymdhmsz**  This folder is composed of three packages which  save the image, lidar and radar data in  a yyMMdd-hhmmsszz format respectivelly.

* **dl_lidar_rgb_radar_matches** This package synchronizes the radar,  lidar and rgb images.

* **dl_sensors2_8_16_gray** This package takes the lidar and radar pointcloud and normilize them either into 8 or 16-depth bits.

* **dl_sensor_fusion_cnn** This node implements the multimodal sensor fusion architecture based on the SegNet CNN.

* **dl_calibration** This node implements the calibration between radar and lidar sensors. 

* **dl_lidar_rgb_radar_syn** This node takes samples for the calibration. 

* **dl_err_opt_lid_rad** This node computes the directional vector.

* **dl_lidar_centerpoint_depth** This node computes the lidar point coordinates x_l , y_l , z_l.


## Prerequisites

The system used to handle the simulations is composed of a L3CAM lidar, a UMRR-96 Type 153 radar, and a GE66 Raider Intel ®Core(TM) i9-10980HK CPU with an NVIDIA GeForce RTX 3070 8Gb GPU. The Robot Operating System (ROS1) Noetic on Ubuntu 20.04.5 LTS is used to collect sensor data, compute extrinsic parameters, and align the sensors. Moreover, the CNN network is simulated in a Jupyter Notebook using Python 3 and a conda environment. 



## Instalation instructions:

*  cd ~/ros/catkin_ws/src 
*  git clone https://github.com/ebernalbeamagine/DeepLearningSensorFusionForPedestrianDetection.git
*  cd ../ 
*  catkin_make 


## Functionality
This tutorial explains how to use the leep learning sensor fusion for pedestrian detection ROS package. 

This repository is divided in two parts: **extrinsic parameters matrix** and **network simulation**.





The **network simulation** runs in two different modes: 

*  **simulator mode**
*  **lidar mode** 

In **simulator mode** we assumed the physical lidar is not connected to the laptop, therefore we  launch a lidar simulator which delivers in an endless loop a pointcloud and a RGB image.




In **lidar mode** we assumed the physical lidar is connected to the laptop.


.







## Run the simulation:

### Extrinsic parameters matrix:


In this mode we assume the lidar is connected to the laptop to get samples for calibration matrix.

In a terminal window we run the roscore command.

**$roscore** 

          
Then, in another terminal window  we launch the radar driver with the following command.



**$rosrun dl_radar_initialization dl_radar_initialization**

                    OR
**$sudo ip link set can0 type can bitrate 500000**

**$sudo ip link set up can0**


The following command  runs the L3CAM ROS wrapper which is an integration between L3CAM and ROS.
          
**$rosrun dl_libl3cam dl_libl3cam**  

$roslaunch dl_lidar_rgb_radar_sync dl_sync_sensors.launch  

        // This launch runs sensor_fusion.launch, automotive_radar_visualization.launch  and the LidarRGB and the LidarPointcloud

           
$rosservice call action "in_: 'pause'"   ////to pause the sync node

$rosservice call action "in_: 'start'"   ////to re start the sync node          
             
///////////////         
///---END---///        
/////////////// 


### lidar mode:

In a terminal window we run the roscore command.

**$roscore** 

Then, in another terminal window  we launch the radar driver with the following command.



**$rosrun dl_radar_initialization dl_radar_initialization**

                    OR
**$sudo ip link set can0 type can bitrate 500000**

**$sudo ip link set up can0**


The following command  runs the L3CAM ROS wrapper which is an integration between L3CAM and ROS.
          
**$rosrun dl_libl3cam dl_libl3cam**  

To be able to see the pointclouds, the rgb image in rviz, the following command can be launched. 

**$roslaunch dl_lidar_ymdhmsz dl_rviz.launch** 

    
The following ROS launch command runs the dl_lidarpointcloud and dl_lidarrgb ROS nodes, which receive lidar and RGB socket data and publish a lidar pointcloud and an RGB image. It also runs the umrr ROS driver. In addition, it runs the dl_lidar_ymdhmsz, dl_radar_ymdhmsz, and dl_image_ymdhmsz nodes, which save the lidar and radar pointclouds and the RGB image in memory in the yyyy-MM-dd:hh:mm:ss:zz format, where yyyy denotes the year, MM denotes the month, dd denotes the day, hh denotes the hour, mm denotes the minute, ss denotes the second, and zz denotes the millisecond. This format was chosen because the lidar, RGB, and radar frequencies are 10 Hz, 6 Hz, and 18 Hz, respectively.

**$roslaunch dl_lidar_ymdhmsz dl_lrgb_sensors.launch**

The next command runs the roeservice that pauses and restart the  dl_lidar_ymdhmsz, dl_radar_ymdhmsz, and dl_image_ymdhmsz nodes. 
      
**$roslaunch dl_lidar_ymdhmsz dl_start.launch**  

# Bibliogrhapy





# NOTE! //--this  repository is still under construction--//


