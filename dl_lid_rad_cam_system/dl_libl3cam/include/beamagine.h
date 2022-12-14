#ifndef BEAMAGINE_H
#define BEAMAGINE_H

#include <stdint.h>
#include <stdbool.h>

typedef enum sensorTypes {
    sensor_lidar = 1,
    sensor_econ_rgb,
    sensor_pol,
    sensor_thermal,
}sensorTypes;

enum imageTypes {
    image_mono = 1,
    image_rgb,
    //!mono polarimetric modes
    pol_co,
    pol_cross,
    pol_docp,
    pol_grad,
    //!rgb polarimetric modes
    pol_raw,
    pol_aolp,
    pol_dolp,
    pol_hsv,
};

typedef enum thermalTypes{
    //!thermal image modes
    thermal_WHITE = 1,
    thermal_BLACK = 17,
    thermal_IRON = 20,
    thermal_COOL = 2,
    thermal_AMBER = 9,
    thermal_INDIGO = 10,
    thermal_TYRIAN = 16,
    thermal_GLORY = 8,
    thermal_ENVY = 16,
    thermal_WHITE_NEW = 100,
    thermal_BLACK_NEW,
    thermal_SPECTRA,
    thermal_PRISM,
    thermal_TYRIAN_NEW,
    thermal_AMBER_NEW,
    thermal_IRON_NEW,
    thermal_HI,
    thermal_HILO,
}thermalTypes;

enum pointCloudColor{
    RAINBOW = 0,
    RAINBOW_Z,
    INTENSITY,
    RGB_FUSION,
    POLARIMETRIC_FUSION,
    POL_PROCESSED_FUSION,
    THERMAL_FUSION,
    RGBT_FUSION
};

enum laserClass{
    laser_class_1 = 0,
    laser_class_3R,
    laser_class_3B
};

typedef enum streamingProtocols {
    protocol_raw_udp = 1,
    protocol_gstreamer,
    protocol_rtsp,
}streamingProtocols;

typedef enum econResolutions{
    reso_640_480 = 1,
    reso_1280_720,
    reso_1920_1080
}econResolutions;

typedef struct sensor{
    streamingProtocols protocol;
    sensorTypes sensor_type;
    uint8_t sensor_status;
    uint8_t image_type;
    bool perception_enabled;
}sensor;

typedef struct l3cam{
    char ip_address[16];
    char serial_number[10];
    uint8_t model;
}l3cam;

#endif // BEAMAGINE_H

