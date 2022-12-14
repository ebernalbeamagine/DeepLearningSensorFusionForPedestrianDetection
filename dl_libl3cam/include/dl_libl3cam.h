#include <ros/ros.h>
#include <libL3Cam.h>
#include <beamagine.h>	
#include <beamErrors.h>
#include "dl_libl3cam/process.h"



/*Standard library*/
using namespace std;

l3cam m_devices[1];
sensor m_sensors[6];

int m_devices_connected;
int m_sensors_connected;
 int error;
 
bool m_device_started;
bool m_device_streaming;
bool flag_process=true;


ros::ServiceServer service;
