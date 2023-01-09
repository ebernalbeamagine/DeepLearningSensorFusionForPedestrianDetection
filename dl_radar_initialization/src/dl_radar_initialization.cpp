

#include <ros/ros.h>
#include <unistd.h>
#include <iostream>
#include <stdexcept>
#include <stdio.h>
#include <string>
#include <sys/wait.h>



/*Standard library*/
using namespace std;



int main (int argc, char** argv){ 
   ros::init (argc, argv, "dl_radar_initialization");
   ros::NodeHandle nh_;
   
   system("sudo ip link set can0 type can bitrate 500000");
   system("sudo ip link set up can0");
   
   
   std::cout<<"\n \n" << "----!!! The Radar node Initialization  is completed !!! --- "<<"\n \n";

  
      
    ros::Rate loop_rate(1);
    while(ros::ok()){
       ros::spinOnce();
       loop_rate.sleep(); 
    }
}
