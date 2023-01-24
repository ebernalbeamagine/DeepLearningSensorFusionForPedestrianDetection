#ifndef udpSender_H
#define udpSender_H


#include <arpa/inet.h>
#include <sys/socket.h>
#include <vector>
#include <string>
#include "dataTypes.h"

class udpSender 
{

public:
    explicit udpSender();

    void setIpAddress(std::string address);

    void setPort(uint16_t port);

    int initializeSocket();

    void sendImage(uint8_t *image_pointer, int16_t image_height, int16_t image_width, uint8_t channels, std::vector<detectionImage> detections, uint32_t timestamp);

    void sendPointcloud(tPointCloudUdp *pointcloud, uint32_t timestamp);

private:
    std::string m_address;
    uint16_t m_port;
    struct sockaddr_in m_socket;
    int m_socket_descriptor;
};

#endif // udpSender_H