umrr_driver
=======================

Overview/Purpose
-----------------------

This package provides a ROS node to convert the target list of a radar sensor into a PointCloud2 message and nodes to further process the point clouds. The main node `umrr_can_publisher` configures the radar sensor and reads the incoming CAN messages.
Also provided is an example for 3D Visualization using Rviz.

### Hardware prerequisites
- Smartmicro automotive radar sensor:
    - UMRR-96 Type 132
    - UMRR-11 Type 153
- Any adapter providing a linux can socket for example:
    - Lawicel USBCAN Adapter
    - PEAK CAN USB Adapter

### Configuration
This driver is designed for Ubuntu 20.04 LTS running ROS Melodic.
Also tested on Ubuntu 16.04 LTS running ROS Kinectic.
CAN Protocol format: json configuration file

### Build Tool
The package uses catkin build tools.

Installation
-----------------------

### CAN socket
The node operates using a can socket. If you are not using a LAWICEL adapter make sure
the socket is reachable for example using the command
`candump slcan0`

The Lawicel adapter can be installed using the following steps:

1. add the kernel module `slcan` to your `/etc/modules` file so it is loaded during bootup
2. check if the module is running by typing
`lsmod|grep slcan`
output should look similar to: `slcan   16834 0`
3. install can-utils via  
`sudo apt install can-utils`
4. Map the usb to a can device by running
`sudo slcand -o -c -f -s6 /dev/ttyUSB0 slcan0`
5. Setup the network device making the socket available for the userspace  
`sudo ifconfig slcan0 up`
Note: You might need the net-tools to run this command (Install them via `sudo apt-get install net-tools`)
6. Test if the setup is done and working correctly by typing  
`candump slcan0`  
You should see the raw can messages

### Virtual environment for python3 (optional)

This node requires python3 while older versions of ROS only support python2.7. Therefore you might have to set up a virtual environment with python3 as follows:

Prerequisites:
install the following packages using apt-get:
- python3-dev
- virtualenvwrapper
- virtualenv

Close your shell afterwards to update your .bash_rc

1. Create an virtual enviroment using:  
`mkvirtualenv -p /usr/bin/python3 <your_enviroment_name>`  
This creates an enviroment and also activates it. Notice the brackets in your command line

2. install the following packages in your virtual enviroment
- rospkg
- catkin_pkg
- crc_mod  

    using the command:  
    `pip install rospkg catkin_pkg crcmod`

Leave your virtual enviroment.

For convenient use e.g. nest the launch file in your existing project, you will have to edit the umrr_can_publisher script.
Open the python script located in umrr_drivers/scripts and edit the shebang line.

Enter the path to the enviroment you just setup. To do this:
1. Open a new terminal
2. type `workon <name_of_your_enviroment>`
3. type `echo $VIRTUAL_ENV` copy the resulting path and leave the console
4. enter the obtained path to your shebang path for example:
`#!/home/user/.virtualenvs/ros_driver/bin/python`


Usage
-----------------------
1. Add this driver package to your workspace and build it via `catkin_make` as usual.
2. Make sure the can socket is available in linux and a sensor is connected. If you are not using socket slcan0, edit the nodes configuration file.
3. Start a launch file:
	- `roslaunch umrr_driver automotive_radar_default.launch`
	
	This will launch the umrr_can_publisher node which will publish the target list of the sensor in a pointcloud2 format. Use this as the minimal setup to read sensor data and
	configure it.
	- `roslaunch umrr_driver automotive_radar_visualization.launch`
	
	This will launch the umrr_can_publisher node and two additional nodes to filter the target list ([pc2_filter](#pc2_filter)) and convert the spherical coordiantes to cartesian coordinates ([spherical_coord_2_cartesian_coord](#spherical_coord_2_cartesian_coord)). These are then visualized in Rviz which is lauched too. Use this setup to get a first impression of the target list and its parameters.

Config files
----------------------

cfg/
- `umrr_driver_default_config.yaml` default setup file for the umrr_can_publisher.
- `pc2filter.cfg` sets the configurable parameters for the rqt server of pc2filter node


Nodes
----------------------

### umrr_can_publisher

This node reads the incoming CAN messages, writes the data to a PointCloud2 message using the `can_spec` and publishes it.

**Important:** This node need Python3, while ROS officially only supports Python2.
So make sure you are starting this node with Python3, for example by using a virtual environment.

#### Published Topics

* **`/radar_data`** (sensor_msgs/PointCloud2)

  A point cloud with the radar targets of the current cycle. Depending on the sensor configuration, the point cloud will contain different fields, which are defined in the corresponding `can_spec`.

#### Services  
- `sensor_status` (srv/sens_status)  
Send a status request to the sensor. Returns the corresponding value. Usage:
`rosservice call /node_namespace/sensor_status section parameter`
- `sensor_parameter` (srv/sens_param)
Send a parameter write to the sensor. Sets the corresponding value. Usage:  
`rosservice call /node_namespace/sensor_parameter section parameter value dim`
- `sensor_parameter_request` (srv/sens_param_req)
Send a parameter request to the sensor. Returns the corresponding value. Usage:  
`rosservice call /node_namespace/sensor_parameter_request section parameter dim`

#### Parameters

* **`~can_spec`** (string)

  Path to the `can_spec` to use.

* **`~frame_id`** (string, default:"radar")

  Frame ID of the sensor.

* **`~can_socket`** (string, default:"slcan0")

  Can socket the node should use.

* **`~legacy_mode`** (int, default: 0, min: 0, max: 1)

  Choose to start node in legacy mode. If set, communication with sensor except for
  receiving a target list is switched off. Also the services `sensor_status` and
  `sensor_parameter` are not offered.

* **`~antenna_mode`** (dict, default:"")

  Provide a dict to configure the sensors used antenna on startup. Node expects following order:
  `section, parameter, value`

* **`~center_frequency`** (dict, default:"")

  Provide a dict to configure the sensors used center frequency on startup. Node expects following order:
  `section, parameter, value`

###  Pc2_filter
A node to filter the targets coming from the sensor. To change values during runtime this
node offers a dynamic reconfigure server.

#### Subscribed Topics

* **`/radar_data`** (sensor_msgs/PointCloud2)

  The input cloud must contain at least the following fields:
  * `Range` (Unit: [m])
  * `Azimuth` (Unit: [°])
  * `Elevation` (Unit: [°])
  * `Speed_Radial` (Unit: [m/s])


#### Published Topics

* **`/filtered_data`** (sensor_msgs/PointCloud2)

    The output cloud contains the same fields as `/radar_data`


### Spherical_Coord_2_Cartesian_Coord

This node converts a point cloud with targets in spherical coordinates to a point cloud with cartesian coordinates,
so that it is possible to view the point cloud in Rviz.

#### Subscribed Topics

* **`filtered_data`** (sensor_msgs/PointCloud2)

  The input cloud must contain the following fields:
  * `Range` (Unit: [m])
  * `Azimuth` (Unit: [°])
  * `Elevation` (Unit: [°])
  * `Speed_Radial` (Unit: [m/s])


#### Published Topics

* **`/target_list_cartesian`** (sensor_msgs/PointCloud2)

  The output cloud contains the following fields:
  * `x` (Unit: [m])
  * `y` (Unit: [m])
  * `z` (Unit: [m])
  * `Speed_Radial` (Unit: [m/s])
  * `RCS` (Unit: [dB])
  * `SNR` (Unit: [dB])


MSE Software Support
----------------------

The driver provides additional support for the MSE software variante of the UMRR-11 Type 153.

### Launch files
It adds a can specification to publish the tracked objects and a new node to visualize these objects. Start up the example launch files as follows to use them:

    roslaunch umrr_driver automotive_mse_default.launch
    roslaunch umrr_driver automotive_mse_visualization.launch

#### Default launch file
The ``automotive_mse_default.launch`` file starts up the smartmicro ROS driver twice, once with the mse specific target list can spec and once with the object list can spec. The published topics can be used by costumer nodes.

#### Visualization launch file
The ``automotive_mse_visualization.launch`` file additionally starts up rviz, the target list node pipeline for visualization of the pointcloud in rviz and a new node to display the tracked objects in rviz as a pointclound too.

Collision Avoidance Software Support
----------------------

The driver provides additional support for the Collision Avoidance software variante of the UMRR-96 Type 132.

### Launch files
It adds a can specification to publish the occupied cell list from the occupancy grid and a new node to visualize the occupied cells. Start up the example launch files as follows to use them:

    roslaunch umrr_driver automotive_collision_avoidance_default.launch
    roslaunch umrr_driver automotive_collision_avoidance_visualization.launch

#### Default launch file
The ``automotive_collision_avoidance_default.launch`` file starts up the smartmicro ROS driver twice, once with the target list can spec and once with the occupied cells list can spec. The published topics can be used by costumer nodes.

#### Visualization launch file
The ``automotive_collision_avoidance_visualization.launch`` file additionally starts up rviz, the target list node pipeline for visualization of the pointcloud in rviz and a new node to display the occupied cells and zones in rviz via markers.

Notes
----------------------
If you have a sensor with software below the required version or a UMRR-8F, you can use
the legacy mode of the driver.

Bugs & Feature Requests
----------------------
Please report bugs and feature requests to: `support@smartmicro.de`
