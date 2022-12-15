#include <ros/ros.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <arpa/inet.h>
#include <stdint.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <pthread.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "sensor_msgs/PointCloud.h"




struct boxImage{
int16_t x;
int16_t y;
int16_t height;
int16_t width;
};
struct detectionImage{
boxImage box;
uint16_t confidence;
};


pthread_t pointcloud_thread;


ros::Publisher lidar_pub_;
ros::Publisher PC2_pub_;

/*Standard library*/
using namespace std;
