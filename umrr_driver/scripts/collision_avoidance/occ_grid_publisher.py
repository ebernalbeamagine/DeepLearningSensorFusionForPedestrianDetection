#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Example Python node to publish on a specific topic."""

# Import required Python code.
import rospy
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import OccupancyGrid
import tf2_ros
import tf2_geometry_msgs
import sensor_msgs.point_cloud2 as pc2


marker_colors = {   0 : {0: [0.0, 0.251, 0.0], 1: [0.0, 0.8, 0.0]},
                    1 : {0: [0.6, 0.302, 0.0], 1: [1.0, 0.5, 0.0]},
                    2 : {0: [0.545, 0.0, 0.0], 1: [1.0, 0.0, 0.0]}}

class OccGridHeightPub:


    def __init__(self):
        
        self.marker_life_time = 0.001
        self.cell_pub = rospy.Publisher("occupied_cells_marker", MarkerArray, queue_size=5)
        if rospy.has_param("~enable_occupied_zone_visualization") and rospy.get_param("~enable_occupied_zone_visualization"):
            self.zone_pub = rospy.Publisher("occupied_zones_marker", MarkerArray, queue_size=5)

        self.occ_sub = rospy.Subscriber("radar_data", pc2.PointCloud2, self.occ_cell_callback)

        self.time = -1
        self.marker_colors = {  0 : {0: [0.0, 0.251, 0.0], 1: [0.0, 0.8, 0.0]},
                                1 : {0: [0.6, 0.302, 0.0], 1: [1.0, 0.5, 0.0]},
                                2 : {0: [0.545, 0.0, 0.0], 1: [1.0, 0.0, 0.0]}}
        self.markers_last_cycle = {}

    
    def occ_cell_callback(self, data):

        property_names = {"Green_Zone_Occupied", "Yellow_Zone_Occupied", "Red_Zone_Occupied", "x", "y", "z", "Warning_Zone", "Warning_Level"}
        markers_current_cycle = {}
        green_zone_occ, yellow_zone_occ, red_occ = 0, 0, 0

        for green_zone_occ, yellow_zone_occ, red_zone_occ, x, y, z, warning_zone, warning_level in pc2.read_points(data, field_names = property_names, skip_nans=True):
            
            # calulate unique marker id based on cell position
            marker_id = int((((y+10)/0.5) * 20) + ((x-0.25)/0.5))

            # create new marker with index
            new_marker = self.create_new_cell_marker(x, y, 0, z, marker_id, warning_zone, warning_level)
            
            if marker_id in self.markers_last_cycle:
                # marker  already exists
                self.markers_last_cycle.pop(marker_id)
                
            markers_current_cycle[marker_id] = new_marker

        # create new cell Marker array
        cell_marker_array = MarkerArray()
        
        # add new and edited markers to array
        for id, marker in markers_current_cycle.items():
            cell_marker_array.markers.append(marker)

        # add markers from last cycle which have not been edited and need to be deleted
        for id, marker  in self.markers_last_cycle.items():
            marker.action = Marker.DELETE
            cell_marker_array.markers.append(marker)

        # publish new, edited and deleted markers
        self.cell_pub.publish(cell_marker_array)

        # save new and edited markers for next cycle
        self.markers_last_cycle.clear()
        self.markers_last_cycle = markers_current_cycle

        if rospy.has_param("~enable_occupied_zone_visualization") and rospy.get_param("~enable_occupied_zone_visualization"):
            self.visualize_occupied_zones(green_zone_occ, yellow_zone_occ, red_zone_occ)


    @staticmethod
    def create_new_cell_marker(x, y, base, height, marker_id, zone_number, warning_level, size_x = 0.3, size_y = 0.3, opacity = 1.0):
        
        try:
            marker_color = marker_colors[zone_number][warning_level]
        except:
            marker_color = [0.5, 0.5, 0.5]

        cube_marker = Marker()
        cube_marker.header.frame_id = "base_link"
        cube_marker.id = marker_id
        # set marker position
        cube_marker.pose.position.x = x 
        cube_marker.pose.position.y = y 
        cube_marker.pose.position.z = (base + height) * 0.5
        cube_marker.pose.orientation.x = 0.0
        cube_marker.pose.orientation.y = 0.0
        cube_marker.pose.orientation.z = 0.0
        cube_marker.pose.orientation.w = 1.0
        # set marker scale
        cube_marker.scale.x = size_x
        cube_marker.scale.y = size_y
        cube_marker.scale.z = max(height, 0.01)
        # set color
        cube_marker.color.a = opacity
        cube_marker.color.r = marker_color[0]
        cube_marker.color.g = marker_color[1]
        cube_marker.color.b = marker_color[2]
        cube_marker.type = Marker.CUBE
        cube_marker.action = Marker.ADD

        return cube_marker      

    def visualize_occupied_zones(self, green_zone_occ, yellow_zone_occ, red_zone_occ):
        zone_marker_array = MarkerArray()

        if rospy.get_param("~enable_zone_size_visualization"):
            warning_level_theshold = rospy.get_param("~warning_level_theshold")

            yellow_zone_p1_x = rospy.get_param("~yellow_zone_p1_x")
            yellow_zone_p1_y = rospy.get_param("~yellow_zone_p1_y")
            yellow_zone_p2_x = rospy.get_param("~yellow_zone_p2_x")
            yellow_zone_p2_y = rospy.get_param("~yellow_zone_p2_y")

            zone_marker_array.markers.append(self.create_new_cell_marker((yellow_zone_p1_x + yellow_zone_p2_x)/2, (yellow_zone_p1_y + yellow_zone_p2_y)/2,
                                             warning_level_theshold, 0.01, 1300, 1, yellow_zone_occ, (yellow_zone_p1_x + yellow_zone_p2_x),
                                             abs(yellow_zone_p1_y - yellow_zone_p2_y), 0.6))

            red_zone_p1_x = rospy.get_param("~red_zone_p1_x")
            red_zone_p1_y = rospy.get_param("~red_zone_p1_y")
            red_zone_p2_x = rospy.get_param("~red_zone_p2_x")
            red_zone_p2_y = rospy.get_param("~red_zone_p2_y")

            zone_marker_array.markers.append(self.create_new_cell_marker((red_zone_p1_x + red_zone_p2_x)/2, (red_zone_p1_y + red_zone_p2_y)/2,
                                             warning_level_theshold, 0.02, 1400, 2, red_zone_occ, (red_zone_p1_x + red_zone_p2_x),
                                             abs(red_zone_p1_y - red_zone_p2_y), 0.6))

        else:
            zone_marker_array.markers.append(self.create_new_zone_icon(-0.6, 0, 0, 0.01, 1000, 0, green_zone_occ))
            zone_marker_array.markers.append(self.create_new_zone_icon(-1.0, 0, 0, 0.01, 1100, 1, yellow_zone_occ))
            zone_marker_array.markers.append(self.create_new_zone_icon(-1.4, 0, 0, 0.01, 1200, 2, red_zone_occ))

        self.zone_pub.publish(zone_marker_array)


    def create_new_zone_icon(self, x, y, base, height, marker_id, zone_number, warning_level):

        zone_marker = self.create_new_cell_marker(x, y, base, height, marker_id, zone_number, warning_level)
        zone_marker.type = Marker.CYLINDER
        return zone_marker     



if __name__ == '__main__':
    rospy.init_node('radar_pub')
    occupied_cells_node = OccGridHeightPub()
    
    rospy.spin()
