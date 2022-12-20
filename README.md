# Deep Learning Sensor Fusion for Pedestrian Detection

This repository presents a fully convolutional neural network architecture for multimodal
sensor fusion, such as lidar, sonar, and RGB camera. The core of the network is based on SegNet,
a pixel-wise semantic segmentation network. A single SegNet is assigned to each sensor reading,
and the outputs are then applied to a fully connected neural network to fuse the three modalities
of sensors. Afterwards, an up-sampling network is applied to recover the fused data. We have
successfully demonstrated the use of semantic segmentation for sensor fusion under the modalities
of three sensors. Moreover, pixel accuracy, loss, and mean intersection over union are the metrics
used for the validation of the accuracy of the network. This package has also proposed an extrinsic
calibration matrix method for sensor alignment.

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

* **dl_err_opt_lid_rad** This node computes the directional vector

* **dl_lidar_centerpoint_depth** This node computes the lidar point coordinates x_l , y_l , z_l

## Getting Started

## Prerequisites

## Functionality


## Instalation instructions:

## Run simulation:

# Bibliogrhapy


