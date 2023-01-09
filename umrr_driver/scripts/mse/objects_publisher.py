#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Example Python node to publish on a specific topic."""

# Import required Python code.
import rospy
from std_msgs.msg import Float32MultiArray
import sensor_msgs.point_cloud2 as pc2

class ObjectsMapper:


    def __init__(self):
        
        self.objects_pub = rospy.Publisher("objects", pc2.PointCloud2, queue_size=5)
        self.objects_sub = rospy.Subscriber("radar_data", pc2.PointCloud2, self.objects_callback)

        self.fields =[pc2.PointField('x',             0, pc2.PointField.FLOAT32, 1),
                    pc2.PointField('y',             4, pc2.PointField.FLOAT32, 1),
                    pc2.PointField('z',             8, pc2.PointField.FLOAT32, 1)]
    
        self.property_names = {"x_Point", "y_Point", "z_Point"}


    def objects_callback(self, data):
        objects = []

        for x, y, z in pc2.read_points(data, field_names = self.property_names, skip_nans=True):
            objects.append([x, y, z])

        cloud_msg = pc2.create_cloud(data.header, self.fields, objects)
        self.objects_pub.publish(cloud_msg)


if __name__ == '__main__':
    rospy.init_node('objects_pub')
    objects_node = ObjectsMapper()
    
    rospy.spin()
