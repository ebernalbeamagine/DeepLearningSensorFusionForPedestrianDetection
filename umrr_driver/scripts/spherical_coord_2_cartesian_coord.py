#!/usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2 as pc2

import tf2_ros
import tf2_geometry_msgs
import PyKDL
from math import cos, sin, radians, sqrt


class CartesianConverter:
    def __init__(self):
        self.fields = [pc2.PointField('x',                  0, pc2.PointField.FLOAT32, 1),
                       pc2.PointField('y',                  4, pc2.PointField.FLOAT32, 1),
                       pc2.PointField('z',                  8, pc2.PointField.FLOAT32, 1),
                       pc2.PointField('Speed_Radial',      12, pc2.PointField.FLOAT32, 1),
                       pc2.PointField('RCS' ,              16, pc2.PointField.FLOAT32, 1),
                      #pc2.PointField('SNR',               20, pc2.PointField.FLOAT32, 1),
                       pc2.PointField('SNR',               20, pc2.PointField.FLOAT32, 1),
                       pc2.PointField('Power',             24, pc2.PointField.FLOAT32, 1),
                       pc2.PointField('vx',                28, pc2.PointField.FLOAT32, 1),
                       pc2.PointField('vy',                32, pc2.PointField.FLOAT32, 1),
                       pc2.PointField('vz',                36, pc2.PointField.FLOAT32, 1)]
        # define publisher for the pointcloud with the cartesian coordinates
        self.pub = rospy.Publisher('target_list_cartesian', pc2.PointCloud2, queue_size=1)
        # define subscriber to pointcloud
        rospy.Subscriber("filtered_data", pc2.PointCloud2, self.pc_callback)

    def pc_callback(self, data):
        points = []
        cloud_points = list(pc2.read_points(data, skip_nans=True, field_names=None))

        # Get the index of the needed fields
        for field, i in zip(data.fields, range(len(data.fields))):
            if field.name == "Range":
                range_index = i
            if field.name =="Azimuth":
                azimuth_index = i
            if field.name == "Elevation":
                elevation_index = i
            if field.name == "Speed_Radial":
                speed_rad_index = i
            if field.name == "RCS":
                rcs_index = i
            if field.name == "Power":
                pow_index = i
            if field.name == "Noise":
                noise_index = i

        for single_point in cloud_points:
            elevation_theta = radians(single_point[elevation_index])
            azimuth_phi = radians(single_point[azimuth_index])
            cos_elev = cos(elevation_theta)

            #x =  single_point[range_index] * cos_elev * cos(azimuth_phi) #single_point[range_index] * cos_elev * cos(azimuth_phi)
            #y =  single_point[range_index] * cos_elev * cos(azimuth_phi) #single_point[range_index] * cos_elev * sin(azimuth_phi)
            #z =   single_point[range_index] * sin(elevation_theta)
            
            x =  single_point[range_index] * cos_elev * cos(azimuth_phi)
            y =  single_point[range_index] * cos_elev * sin(azimuth_phi)
            z =   -single_point[range_index] * sin(elevation_theta)
            snr = single_point[pow_index] - single_point[noise_index]
            
            #std::cout<<"point  "<< single_point[speed_rad_index] << "\n"
            
            vx=single_point[speed_rad_index] * cos_elev * cos(azimuth_phi)
            vy=single_point[speed_rad_index] * cos_elev * sin(azimuth_phi)
            vz=single_point[speed_rad_index] * cos_elev 

            #points.append([x, y, z, single_point[speed_rad_index], single_point[rcs_index], snr])
            points.append([x, y, z, single_point[speed_rad_index], single_point[rcs_index], snr,single_point[pow_index],vx,vy,vz])
            #points.append([vx, y, z, single_point[speed_rad_index], single_point[rcs_index], snr,single_point[pow_index]])
        
        cloud_msg = pc2.create_cloud(data.header, self.fields, points)
        self.pub.publish(cloud_msg)


if __name__ == '__main__':
    rospy.init_node('target_list_2_pointcloud', anonymous=True)
    tl2cart = CartesianConverter()
    rospy.spin()
