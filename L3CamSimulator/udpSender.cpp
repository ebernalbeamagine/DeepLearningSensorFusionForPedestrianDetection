#include "udpSender.h"

#include <iostream>
#include <cstring>

udpSender::udpSender()
{
    m_port = 6000;
    m_address = "127.0.0.1";
}

void udpSender::setIpAddress(std::string address)
{
    m_address = address;
}

void udpSender::setPort(uint16_t port)
{
    m_port = port;
}

int udpSender::initializeSocket()
{
    if( (m_socket_descriptor = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1){
        std::cout<<"udpSender::initializeSocket Socket Error\n";
        return 1;
    }

    memset((char *) &m_socket, 0, sizeof(struct sockaddr_in));

    m_socket.sin_addr.s_addr = inet_addr((char *)m_address.c_str());

    m_socket.sin_family = AF_INET;
    m_socket.sin_port = htons((int)m_port);

    if(inet_aton((char*)m_address.c_str(), &m_socket.sin_addr) == 0){
        std::cout<<"udpSender::initializeSocket Inet_aton() failed \n";
        return 2;
    }

    return 0;
}

void udpSender::sendImage(uint8_t *image_pointer, int16_t image_height, int16_t image_width, uint8_t channels,
                        std::vector<detectionImage> detections, uint32_t timestamp){


    int buffer_size = image_height*image_width*channels;
    uint8_t *image_buffer = (uint8_t*)malloc(buffer_size);

    memcpy(image_buffer, image_pointer, buffer_size);

    int32_t size_to_send = 8000;

    int32_t n_pack = buffer_size  / size_to_send;
    int32_t last_size = buffer_size % size_to_send;

    char stx = 0x02;
    char etx = 0x03;

    char *first_data = (char*)malloc(11);
    first_data[0] = stx;

    memcpy(&first_data[1], &image_height, 2);
    memcpy(&first_data[3], &image_width, 2);

    first_data[5] = channels;
    memcpy(&first_data[6], &timestamp, 4);
    first_data[10] = detections.size();

    //!send the detections in a new message
    socklen_t socket_len = sizeof(struct sockaddr_in);
    if(sendto(m_socket_descriptor, first_data, 11, 0, (struct sockaddr *)&m_socket, socket_len) == -1){
        std::cout<<"Error sending STX package";
    }

    free(first_data);

    //!enviar detecciones
    for(int i=0; i<detections.size(); ++i){

        //!size of detection 10
        char *detection_data = (char*)malloc(10*sizeof(char));
        memcpy(&detection_data[0], &detections.at(i).confidence, 2);
        memcpy(&detection_data[2], &detections.at(i).box.x, 2);
        memcpy(&detection_data[4], &detections.at(i).box.y, 2);
        memcpy(&detection_data[6], &detections.at(i).box.height, 2);
        memcpy(&detection_data[8], &detections.at(i).box.width, 2);

        if(sendto(m_socket_descriptor, detection_data, 10, 0, (struct sockaddr*)&m_socket, socket_len) == -1){
            std::cout<<"Error sending detection package";
        }

        free(detection_data);
    }

    //!enviar imagen
    for(int i=0; i < n_pack; ++i){

        char *data = (char*)malloc(size_to_send);

        memcpy(data, &image_buffer[i*size_to_send], size_to_send);

        if(sendto(m_socket_descriptor, data, size_to_send, 0, (struct sockaddr *)&m_socket, socket_len) == -1){
            std::cout<<"Error sending image package";
        }

        free(data);
    }

    if(last_size > 0){

        char *data = (char*)malloc(last_size);

        int offset = n_pack * size_to_send;

        memcpy(data, &image_buffer[offset], last_size);

        if(sendto(m_socket_descriptor, data, last_size, 0, (struct sockaddr *)&m_socket, socket_len) == -1){
            std::cout<<"Error sending last image package";
        }
        free(data);
    }

    //!end of image
    if(sendto(m_socket_descriptor, &etx, 1, 0, (struct sockaddr*)&m_socket, socket_len) == -1){
        std::cout<<"Error sending ETX data";
    }

    free(image_buffer);
}


void udpSender::sendPointcloud(tPointCloudUdp *pointcloud, uint32_t timestamp){

    int32_t points_to_send = (pointcloud->size - 2)/5;
    int n_pack = 1;
    char stx = 0x02;
    char etx = 0x03;

    char first_data[17];
    first_data[0] = stx;
    memcpy(&first_data[1], &points_to_send, 4);
    memcpy(&first_data[5], &pointcloud->data_buffer[0], 4);
    memcpy(&first_data[9], &pointcloud->data_buffer[1], 4);
    memcpy(&first_data[13], &timestamp, 4);

    socklen_t socket_len = sizeof(struct sockaddr_in);
    if(sendto(m_socket_descriptor, first_data, 17, 0, (struct sockaddr *)&m_socket, socket_len) == -1){
        std::cout<<"Error sending package";
    }

    int32_t points = 400;
    int size_to_send = (points*20) + 4;

    n_pack = points_to_send / points;
    int32_t last_size = points_to_send % points;
    int buf_cnt = 2;
    for(int i=0; i < n_pack; ++i){

        char *data = (char*)malloc(size_to_send);
        memcpy(&data[0], &points, 4);

        for(int j=0;j<points; ++j){

            memcpy(&data[4 + ((20*j))], &pointcloud->data_buffer[buf_cnt], 4);
            memcpy(&data[4 + ((20*j)+4)], &pointcloud->data_buffer[buf_cnt+1], 4);
            memcpy(&data[4 + ((20*j)+8)], &pointcloud->data_buffer[buf_cnt+2], 4);
            memcpy(&data[4 + ((20*j)+12)], &pointcloud->data_buffer[buf_cnt+3], 4);
            memcpy(&data[4 + ((20*j)+16)], &pointcloud->data_buffer[buf_cnt+4], 4);

            buf_cnt+=5;

        }

        if(sendto(m_socket_descriptor, data, size_to_send, 0, (struct sockaddr*)&m_socket, socket_len) == -1){
            std::cout<<"Error sending data";
        }
        free(data);

    }

    if(last_size > 0){
        int buff_size = (last_size*20) + 4;
        char *data = (char*)malloc(buff_size);
        memcpy(&data[0], &last_size, 4);

        for(int32_t j=0;j<last_size; ++j){

            memcpy(&data[4 + ((20*j))],    &pointcloud->data_buffer[buf_cnt], 4);
            memcpy(&data[4 + ((20*j)+4)],  &pointcloud->data_buffer[buf_cnt+1], 4);
            memcpy(&data[4 + ((20*j)+8)],  &pointcloud->data_buffer[buf_cnt+2], 4);
            memcpy(&data[4 + ((20*j)+12)], &pointcloud->data_buffer[buf_cnt+3], 4);
            memcpy(&data[4 + ((20*j)+16)], &pointcloud->data_buffer[buf_cnt+4], 4);

            buf_cnt +=5;

        }

        if(sendto(m_socket_descriptor, data, buff_size, 0, (struct sockaddr*)&m_socket, socket_len) == -1){
            std::cout<<"Error sending data";
        }
        free(data);
    }

    if(sendto(m_socket_descriptor, &etx, 1, 0, (struct sockaddr*)&m_socket, socket_len) == -1){
        std::cout<<"Error sending data\n";
    }


}
