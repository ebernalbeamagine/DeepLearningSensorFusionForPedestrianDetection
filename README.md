# Deep Learning Sensor Fusion for Pedestrian Detection

This repository presents a fully convolutional neural network architecture for multimodal sensor fusion, such as lidar, sonar, and RGB camera for pedestrian detection. The sensor fusion methodology is based on [[1]](https://doi.org/10.1117/12.2587993). And, The core of the network is based on SegNet [[2]](https://ieeexplore.ieee.org/document/7803544), a pixel-wise semantic segmentation network. A single SegNet is assigned to each sensor reading, and the outputs are then applied to a fully connected neural network to fuse the three modalities of sensors. Afterwards, an up-sampling network is applied to recover the fused data.


This repository  also presents an extrinsic calibration matrix method for sensor alignment based on singular value decomposition. A calibration board, proposed in [25], has been chosen as inspiration. It consists of three rectangular styrofoam pieces laid side by side on top of each other. The middle layer contains four circles that serve as edge detectors for the lidar and camera sensors. Furthermore, a trihedral corner reflector is placed in the back of the calibration board at the intersection of the four circles. It is worth mentioning that the styrofoam does not affect the detection of the radarcsignal when it is reflected by the corner reflector.

The layout of our  calibration board is illustrated in Figure ![3](https://github.com/ebernalbeamagine/DeepLearningSensorFusionForPedestrianDetection/blob/master/documents/cb22.jpeg), where two pieces of styrofoam are used, the centers of the circles are used as a four point descriptors which intersection gives a point coordinate (x,y) of the location of the laser that matches the position of the trihedral corner reflector. The dash black line shows the place of the corner
reflector. Moreover, a single copper-plated trihedral corner reflector was made as it is shown in the Figure 4. The dimensions are chosen according to [29] and are such that the single areas of the corner reflector are larger compared to the radar wavelength. Thus, the side length edge √
ç of the three isosceles triangles (a) is chosen to be 14 cm and
the base of the triangles L = a*sqrt(2) gives 19.7 cm.


<a href="https://www.youtube.com/watch?v=HpYqldQJJ2Q&feature=youtu.be" target="_blank"><img src="http://i3.ytimg.com/vi/HpYqldQJJ2Q/hqdefault.jpg" 
alt="IMAGE ALT TEXT HERE" width="480" height="360" border="10" /></a>

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

This repository is divided in three parts: **data acquisition,** **extrinsic parameters matrix** and **network simulation**.





The **data acquisition** runs in three different modes: 

*  **simulator mode**
*  **lidar mode** 
*  **sample calibration**

In **simulator mode** we assumed the physical lidar is not connected to the laptop, therefore we  launch a lidar simulator which delivers in an endless loop a pointcloud and a RGB image.


    

    Move the L3CamSimulator to catkin_ws

    cd ~/catkin_ws/L3CamSimulator/build

    $./l3cam_simulator

    cd ~/catkin_ws

    roslaunch dl_lidar_ymdhmsz dl_lrgb_sensors.launch

    roslaunch dl_lidar_ymdhmsz dl_rviz.launch

    roslaunch dl_lidar_ymdhmsz dl_start.launch


In **lidar mode** we assumed the physical lidar is connected to the laptop.

    cd ~/catkin_ws

    $roscore

    $rosrun dl_radar_initialization dl_radar_initialization

                    OR

    $sudo ip link set can0 type can bitrate 500000

    $sudo ip link set up can0
   
    $rosrun dl_libl3cam dl_libl3cam  

    $roslaunch dl_lidar_ymdhmsz dl_rviz.launch     

    $roslaunch dl_lidar_ymdhmsz dl_lrgb_sensors.launch
      
    $roslaunch dl_lidar_ymdhmsz dl_start.launch  


In **sample calibration** the lidar is connected to the laptop to get samples for calibration

    $roscore 
          
    $rosrun dl_libl3cam dl_libl3cam     
    
    $roslaunch dl_lidar_rgb_radar_sync dl_sync_sensors.launch

    $rosservice call action "in_: 'pause'"

    $rosservice call action "in_: 'start'"
 







**Extrinsic parameters matrix**  or **sensor calibration**  is the process of finding the extrinsic parameters between a multi-modal L3CAM lidar-RGB sensor from the company Beamagine  and the UMRR-96 Type 153 radar from the company Smartmicro. In a nutshell, is to find a matrix ** T ** that best aligns both lidar and radar data points.


    $rosrun dl_err_opt_lid_rad  dl_err_opt_lid_rad

    $rosrun dl_lidar_centerpoint_depth dl_lidar_centerpoint_depth 

    dl_calibration.ipynb


The **network simulation**  is runmed  in a Jupyter-Notebook using Python 3
and a conda environment.

    dl_SegNet.ipynb






# Bibliogrhapy

1. Rawashdeh, N.; Bos, J.; Abu-Alrub, N. Drivable path detection using CNN sensor fusion for autonomous driving in the snow. 2021, p. 5. https://doi.org/10.1117/12.2587993.

2. Badrinarayanan, V.; Kendall, A.; Cipolla, R. SegNet: A Deep Convolutional Encoder-Decoder Architecture for Image Segmentation. IEEE Transactions on Pattern Analysis and Machine Intelligence 2017, 39, 2481–2495.  https:/doi.org/10.1109/TPAMI.2016.2644615.



# NOTE! //--this  repository is still under construction--//


