#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <libL3Cam.h>
#include <beamagine.h>
#include <beamErrors.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <stdint.h>
#include <arpa/inet.h>
#include <sys/socket.h>

pthread_t image_thread_handler;

ros::Publisher img_pub_;

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



void error(const char *msg){

   perror(msg);

}

/*Standard library*/
using namespace std;
