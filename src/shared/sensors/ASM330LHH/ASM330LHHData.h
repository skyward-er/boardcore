#ifndef ASM330LHH_DATA
#define ASM330LHH_DATA

struct asm330lhh_data {
    float gyro_x;           // Gyroscope x axis
    float gyro_y;           // Gyroscope y axis
    float gyro_z;           // Gyroscope z axis
    float accel_x;          // Accelerometer x axis
    float accel_y;          // Accelerometer y axis
    float accel_z;          // Accelerometer z axis
    float temperature;       // Temperature
};

#endif