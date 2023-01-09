#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import tf2_geometry_msgs
import PyKDL
import sensor_msgs.point_cloud2 as pc2
from math import pi, sin
import numpy as np

from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from umrr_driver.cfg import pc2filterConfig as ConfigType


class pc2_filter:

    def __init__(self):
        # Create a dynamic reconfigure server.
        self.reconf_server = dyn_reconfigure()
        # setup subscriber to spherical coordinates
        self.sub = rospy.Subscriber("radar_data", pc2.PointCloud2,self.mod_pcl, queue_size=1)
        # setup publisher for spherical coordinates
        self.pub = rospy.Publisher('filtered_data', pc2.PointCloud2, queue_size=1)

    def mod_pcl(self, cloud):
        # setup temporary point list
        points = []
        # read in all information from msg
        for field, i in zip(cloud.fields, range(len(cloud.fields))):
            if field.name == "Azimuth":
                azimuth_index = i
            if field.name == "Range":
                range_index = i
            if field.name == "Elevation":
                elevation_index = i
            if field.name == "Speed_Radial":
                sp_index = i
            if field.name == "RCS":
                rcs_index = i
            if field.name == "Cycle_Duration":
                cycled_dur_index = i
            if field.name == "Number_Of_Objects":
                num_object_index = i
            if field.name == "Noise":
                noise_index = i
            if field.name == "Power":
                power_index = i

        for point in pc2.read_points(cloud):
            # apply selected filter value
            if self.reconf_server.filter_range and (point[range_index] <= self.reconf_server.min_range or point[range_index] >= self.reconf_server.max_range):
                    continue

            if self.reconf_server.filter_azimuth and (point[azimuth_index] <= self.reconf_server.min_azimuth or point[azimuth_index] >= self.reconf_server.max_azimuth):
                    continue

            if self.reconf_server.filter_elevation and (point[elevation_index] <= self.reconf_server.min_elevation or point[elevation_index] >= self.reconf_server.max_elevation):
                    continue
                    
            if self.reconf_server.filter_speed and (point[sp_index] <= self.reconf_server.min_radial_speed or point[sp_index] >= self.reconf_server.max_radial_speed):
                    continue
            
            if self.reconf_server.filter_rcs and (point[rcs_index] <= self.reconf_server.min_rcs or point[rcs_index] >= self.reconf_server.max_rcs):
                    continue
                    
            if self.reconf_server.filter_snr:
                snr_value = point[power_index] - point[noise_index]
                if snr_value < self.reconf_server.min_snr or snr_value > self.reconf_server.max_snr:
                    continue
            points.append(point)

        # create cloud message
        cloud_msg = pc2.create_cloud(cloud.header, cloud.fields, points)
        # publish message
        self.pub.publish(cloud_msg)

    def get_interpolated_snr(self, target_range):
        # build arrays
        xp = [0, self.reconf_server.ef_range]
        fp = [self.reconf_server.heigh_thresh, self.reconf_server.low_thresh]

        return np.interp(target_range, xp, fp)

class dyn_reconfigure:

    def __init__(self):
        # setup up dynamic reconfigure server
        self.server = DynamicReconfigureServer(ConfigType, self.reconfigure_cb)

    def reconfigure_cb(self, config, dummy):
        """Create a callback function for the dynamic reconfigure server."""
        # Fill in local variables with values received from dynamic reconfigure
        self.filter_range = config["enable_filter_range"]
        self.min_range = config["min_range"]
        self.max_range = config["max_range"]
        
        self.filter_azimuth = config["enable_azimuth_filter"]
        self.min_azimuth = config["min_azimuth_angle"]
        self.max_azimuth = config["max_azimuth_angle"]

        self.filter_elevation = config["enable_elevation_filter"]
        self.min_elevation = config["min_elevation_angle"]
        self.max_elevation = config["max_elevation_angle"]

        self.filter_speed = config["enable_radial_speed_filter"]
        self.min_radial_speed = config["min_radial_speed"]
        self.max_radial_speed = config["max_radial_speed"]

        self.filter_rcs = config["enable_rcs_filter"]
        self.min_rcs = config["min_rcs"]
        self.max_rcs = config["max_rcs"]

        self.filter_snr = config["enable_snr_filter"]
        self.min_snr = config["min_snr"]
        self.max_snr = config["max_snr"]

        return config


# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('pointcloud_filter')
    # Go to class functions that do all the heavy lifting.
    pcl = pc2_filter()
    # Allow ROS to go to all callbacks.
    rospy.spin()
