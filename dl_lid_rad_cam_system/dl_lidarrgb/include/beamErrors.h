/*
LIST OF BEAMAGINE ERRORS
*/

#define L3CAM_OK    0

//!INTERNAL ERRORS
#define L3CAM_TIMEOUT_ERROR                     210
#define L3CAM_ERROR_OUT_OF_MEMORY               211

#define L3CAM_UNDEFINED_LASER_CLASS             220
#define L3CAM_NO_SENSORS_AVAILABLE              21
#define L3CAM_UNDEFINED_SENSOR                  222
#define L3CAM_VALUE_OUT_OF_RANGE                223
#define L3CAM_SENSOR_NOT_AVAILABLE              224

#define L3CAM_ERROR_CREATING_TCP_CLIENT_SOCKET  230
#define L3CAM_ERROR_INITALIZING_WSA             231
#define L3CAM_ERROR_CONNECTING_WITH_TCP_SERVER  232
#define L3CAM_ERROR_SENDING_TCP_MESSAGE         233
#define L3CAM_ERROR_CREATING_TCP_SERVER_SOCKET  234
#define L3CAM_ERROR_BINDING_SOCKET              235
#define L3CAM_ERROR_STARTING_TCP_SERVER         236
#define L3CAM_ERROR_ACCEPTING_TCP_CLIENT        237

//!LIDAR ERRORS


//!POLARIMETRIC ERRORS
#define ERROR_POL_STD_EXCEPTION             30
#define ERROR_POL_RUNTIME_EXCEPTION         31
#define ERROR_POL_GENERIC_EXCEPTION         32
#define ERROR_POL_TIMEOUT_EXCEPTION         33
#define ERROR_POL_UNDEFINED_ERROR           34
#define ERROR_POL_PERCEPTION_EXCEPTION      35

//!THERMAL ERRORS
#define ERROR_OPENING_THERMAL_CAMERA    40
#define ERROR_THERMAL_IMAGE_SIZE        41
#define ERROR_CLOSING_THERMAL_CAMERA    42
#define ERROR_SETTING_THERMAL_LUT       43
#define ERROR_GETTING_THERMAL_LUT       44
#define ERROR_SETTING_THERMAL_SHARPEN   45
#define ERROR_SETTING_THERMAL_SMOOTH    46
#define ERROR_GETTING_THERMAL_SYSTEM    47
#define ERROR_SETTING_THERMAL_SYSTEM    48

/*
#define BEAM_ERROR_READING_CONFIGURATION_FILE   100
#define BEAM_ERROR_WIRITING_CONFIGURATION_FILE  101
#define BEAM_ERROR_READING_CALIBRATION_FILE     102
#define BEAM_ERROR_IN_LOCAL_IP                  103
#define BEAM_ERROR_IN_LIDAR_IP                  104
#define BEAM_ERROR_CREATING_UDP_SOCKET          105
#define BEAM_ERROR_IN_CONVERSION_UDP_SOCKET     106
#define BEAM_ERROR_BINDING_SOCKET               107
#define BEAM_ERROR_SENDING_UDP_PACKAGE          108
#define BEAM_ERROR_STARTING_CAMERA              109
#define BEAM_ERROR_STOPING_CAMERA               110
#define BEAM_ERROR_READING_UDP_DATA             111
#define BEAM_ERROR_CONFIGURING_SOCKET           112
#define BEAM_ERROR_INITIALIZING_MEMORY          113
#define BEAM_ERROR_OPERATION_CODE_NOT_DEFINED   114
#define BEAM_ERROR_VALUE_NOT_CORRECT            115
#define BEAM_ERROR_OPENING_CONFIGURATION_FILE   116
#define BEAM_ERROR_OPENING_CALIBRATION_FILE     117
#define BEAM_ERROR_READING_CORRECTION_FILE      118
//!----------MIPI V4L2 ERRORS
#define BEAM_ERROR_OPENING_MIPI_DEVICE              119
#define BEAM_ERROR_SETTING_MIPI_FORMAT              120
#define BEAM_ERROR_SETTING_NUMBER_TDC               121
#define BEAM_ERROR_SETTING_NUMBER_OF_LINES          122
#define BEAM_ERROR_SETTING_PACKET_SIZE              123
#define BEAM_ERROR_SETTING_LAST_PACKET_SIZE         124
#define BEAM_ERROR_SETTING_TDC_LINE_SIZE            125
#define BEAM_ERROR_SETTING_NUMBER_OF_PACKETS        126
#define BEAM_ERROR_SETTING_NUMBER_OF_HITS           127 
#define BEAM_ERROR_SETTING_POINTS_IN_DATA_PACKET    128
#define BEAM_ERROR_SETTING_POINTS_LAST_DATA_PACKET  129
#define BEAM_ERROR_SETTING_CONFIGURATION_DONE       130
#define BEAM_ERROR_MIPI_S_STREAMING                 131
#define BEAM_ERROR_INITIALIZING_MIPI_PARAMS         132
#define BEAM_ERROR_READING_MIPI_DATA                133
#define BEAM_ERROR_MIPI_DEVICE_ALREADY_OPEN         134
#define BEAM_ERROR_MIPI_UNABLE_TO_OPEN_DEVICE       135
//!---------SPI ERRORS
#define BEAM_COMMS_SPI_OK  = 1
#define BEAM_COMMS_LIDAR_MANAGER_POWER_ON_DIAGNOSTICS_FAILURE   -1
#define BEAM_COMMS_LIDAR_MANAGER_TDCS_TEST_PATTERNS_TESTS_FAILURE  -2
#define BEAM_COMMS_LIDAR_MANAGER_SPI_OPEN_FAILURE  -3
#define BEAM_COMMS_LIDAR_MANAGER_CONFIG_DMD_FAILURE  -5
#define BEAM_COMMS_LIDAR_MANAGER_TDC_TEST_PATTERN_FAILURE  -6
#define BEAM_COMMS_LIDAR_MANAGER_TDC_NOT_IN_CFG_STATE  -7
#define BEAM_COMMS_SAFETY_ALERT  -8
#define BEAM_COMMS_LIDAR_MANAGER_PARAMETER_NOT_CONFIGURABLE_IN_RUNTIME  -9
#define BEAM_COMMS_SPI_OPEN_ERROR  -100
#define BEAM_COMMS_SPI_IOCTL_ERROR   -101
#define BEAM_COMMS_SPI_DATA_OUT_OF_RANGE  -102
#define BEAM_COMMS_SPI_COMMAND_FEEDBACK_ERROR  -103
#define BEAM_COMMS_SPI_READ_REGISTER_ERROR  -104
#define BEAM_COMMS_SPI_WRITE_REGISTER_ERROR  -105
#define BEAM_COMMS_LIDAR_CONFIG_FILE_OPEN_ERROR  -200
#define BEAM_COMMS_LIDAR_CONFIG_INVALID_PARAMETER  -201
#define BEAM_COMMS_LIDAR_CONFIG_WRONG_NUMBER_OF_DMD_PATTERNS  -202
#define BEAM_COMMS_LIDAR_CONFIG_WRONG_APD  -203
#define BEAM_COMMS_LIDAR_CONFIG_WRONG_MONITORING  -204
#define BEAM_COMMS_LIDAR_CONFIG_WRONG_PIN_START  -205
#define BEAM_COMMS_LIDAR_CONFIG_WRONG_DMD  -206
#define BEAM_COMMS_LIDAR_CONFIG_WRONG_SCANNING_MIRROR  -207
#define BEAM_COMMS_LIDAR_CONFIG_SCANNING_MIRROR_VOLTAGE_RANGE_EXCEEDED  -208
#define BEAM_COMMS_LIDAR_CONFIG_FAILED_TO_ALLOCATE_MEMORY_FOR_FAST_AXIS_LUTS  -209
#define BEAM_COMMS_SCANNING_MIRROR_0_DAC_NOT_CONFIGURED  -301
#define BEAM_COMMS_SCANNING_MIRROR_1_DAC_NOT_CONFIGURED  -302
#define BEAM_COMMS_TDC0_TEST_FAILED  -401
#define BEAM_COMMS_TDC0_UNEXPECTED_STATE  -402
#define BEAM_COMMS_TDC0_UNKNOWN_STATE  -403
#define BEAM_COMMS_TDC1_TEST_FAILED      -404
#define BEAM_COMMS_TDC1_UNEXPECTED_STATE  -405
#define BEAM_COMMS_TDC1_UNKNOWN_STATE  -406
*/