#pragma once

#include <inttypes.h>

typedef struct tPointCloudUdp{
    int32_t size;
    int32_t *data_buffer;
}tPointCloudUdp;


struct boxImage{
    int16_t x;
    int16_t y;
    int16_t height;
    int16_t width;
};

struct detectionImage{
    boxImage box;
    uint16_t confidence;
    uint16_t label;
};