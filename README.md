# PatrolBot Self Driving main repository.

This repository presents a fully convolutional neural network architecture for multimodal
sensor fusion, such as lidar, sonar, and RGB camera. The core of the network is based on SegNet,
a pixel-wise semantic segmentation network. A single SegNet is assigned to each sensor reading,
and the outputs are then applied to a fully connected neural network to fuse the three modalities
of sensors. Afterwards, an up-sampling network is applied to recover the fused data. We have
successfully demonstrated the use of semantic segmentation for sensor fusion under the modalities
of three sensors. Moreover, pixel accuracy, loss, and mean intersection over union are the metrics
used for the validation of the accuracy of the network. This work has also proposed an extrinsic
calibration matrix method for sensor alignment.

## Package description:

* **dl_lid_rad_cam_system** This package contains three sub-folders