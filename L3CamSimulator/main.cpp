#include <iostream>
#include "dataTypes.h"
#include "udpSender.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include <unistd.h>
#include <pthread.h>
#include <vector>

#define BMG_UNUSED(x) (void)x;

std::string m_image_path = "191811387_original.png";//"93751405.png";//;
std::string m_pointcloud_path = "1233477703_original.bin"; //"93751112.bin"; //

//!IMAGE DATA
cv::Mat m_rgb_image;

//!POINTCLOUD DATA
uint32_t number_of_points;
tPointCloudUdp m_point_cloud; 

pthread_t send_image_thread_handler;
pthread_t send_pointcloud_thread_handler;

udpSender m_image_sender;
udpSender m_pointcloud_sender;


void *sendImagehread(void *functionData);

void *sendPointCloudThread(void *functionData);

//!thread para enviar pointcloud
void *sendPointCloudThread(void *functionData)
{
    BMG_UNUSED(functionData);

    uint32_t timestamp = 154526187;

    while(true){

        m_pointcloud_sender.sendPointcloud(&m_point_cloud, timestamp);

        usleep(160000);
    }
    
    return 0;
}

//!thread para enviar imagen
void *sendImageThread(void *functionData)
{
    BMG_UNUSED(functionData);
    
    std::vector<detectionImage> detections;
    detections.clear();
    uint32_t timestamp = 154526587;

    while(true){

        m_image_sender.sendImage((uint8_t*)m_rgb_image.data, (uint16_t)m_rgb_image.rows, (uint16_t)m_rgb_image.cols,
                                 (uint8_t)3, detections, timestamp);
  
        usleep(100000);
    }
    
    return 0;
}

void readPointCloud(){

    FILE *pointcloud_file = std::fopen(m_pointcloud_path.c_str(), "rb");

    std::fread(&number_of_points, sizeof(uint32_t), 1, pointcloud_file);
    int32_t value;

    m_point_cloud.size = (number_of_points * 5) + 2;
    m_point_cloud.data_buffer = (int32_t*)malloc(sizeof(int32_t)*m_point_cloud.size);
    m_point_cloud.data_buffer[0] = 3542;
    m_point_cloud.data_buffer[1] = 2548;

    int pos = 2;
    for(int i=0; i<number_of_points; ++i){
        std::fread(&value, sizeof(int32_t), 1, pointcloud_file);
        m_point_cloud.data_buffer[pos] = value;
        ++pos;
        std::fread(&value, sizeof(int32_t), 1, pointcloud_file);
        m_point_cloud.data_buffer[pos] = value;
        ++pos;
        std::fread(&value, sizeof(int32_t), 1, pointcloud_file);
        m_point_cloud.data_buffer[pos] = value;
        ++pos;
        std::fread(&value, sizeof(int32_t), 1, pointcloud_file);
        m_point_cloud.data_buffer[pos] = value;
        ++pos;
        std::fread(&value, sizeof(int32_t), 1, pointcloud_file);
        m_point_cloud.data_buffer[pos] = value;
        ++pos;
    }

    std::fclose(pointcloud_file);

    }

int main(){

    //!leer imagen
    m_rgb_image = cv::imread(m_image_path, cv::IMREAD_COLOR);

    //!leer pointcloud
    readPointCloud();

    //!inicializar senders
    m_image_sender.setIpAddress("127.0.0.1");
    m_image_sender.setPort(6020);
    m_image_sender.initializeSocket();

    m_pointcloud_sender.setIpAddress("127.0.0.1");
    m_pointcloud_sender.setPort(6050);
    m_pointcloud_sender.initializeSocket();

    //!lanzar hilo de imagen
    pthread_create (&send_image_thread_handler, NULL, sendImageThread, NULL);

    //!lanzar hilo de pointcloud
    pthread_create (&send_pointcloud_thread_handler, NULL, sendPointCloudThread, NULL);

    //!lanzar hilo de app
    while(1){
        usleep(1000000); //!waits 1 sec
    }

}
