/*  LSM6DS3H Driver
 *
 * Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Nuno Barcellos
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef LSM6DS3H_H
#define LSM6DS3H_H

#include <drivers/BusTemplate.h>

#include "../Sensor.h"
#include "Common.h"
#include "miosix.h"

/** ********ATTENTION***********
 * @warning Once upon a time, this driver was working, but due to hardware
 * problems we were unable to confirm it and test it properly. If you need this,
 * you should test it a bit before using it in production code.
 */

template <typename Bus>
class LSM6DS3H : public GyroSensor,
                 public AccelSensor,
                 public CompassSensor,
                 public TemperatureSensor
{
#pragma pack(1)
    // __extension__ is needed to prevent compiler warnings for anonymous
    // structs
    typedef union {
        __extension__ struct
        {
            int16_t temp;
            int16_t gyro[3];
            int16_t accel[3];
        };
        int16_t buf[8];
    } lsmData_t;

#pragma pack()

public:
    LSM6DS3H(uint8_t accelFullScale, uint16_t gyroFullScale)
    {
        accelFS = accelFullScale;
        gyroFS  = gyroFullScale;
    }

    bool init() override
    {
        // SPI clock frequency up to 10 MHz
        // The device is compatible with SPI modes 0 and 3

        uint8_t whoami = Bus::read(RegMap::WHO_AM_I);
        printf("[LSM] expected: %x actual: %x\n", whoami_value, whoami);
        if (whoami != whoami_value)
        {
            last_error = ERR_NOT_ME;
            return false;
        }

        // Reset device
        uint8_t reg = Bus::read(RegMap::CTRL3_C);
        Bus::write(reg | 0x01);

        miosix::Thread::sleep(100);

        // clang-format off
        uint8_t init_data[][2] = 
        {
            {RegMap::CTRL3_C, 0x44}, // Register address automatically incremented during a multiple 
                                     // byte access with a serial interface; LSB @ lower address;
                                     // SPI 4 wire; Output registers not updated until MSB and LSB
                                     // have been read
            {RegMap::CTRL1_XL, (uint8_t) (0x70 | (accelFS << 2))}, // Accel ODR to 833 Hz,
                                                                   // Anti-aliasing filter to 400 hz
            // {RegMap::CTRL5_C, 0x60}, // Rounding read enabled for both accel and gyro
            {RegMap::CTRL2_G, (uint8_t) (0x70 | (gyroFS << 2))},   // Gyro ODR to 833 Hz
        };
        // clang-format on

        for (size_t i = 0; i < sizeof(init_data) / sizeof(init_data[0]); i++)
            Bus::write(init_data[i][0], init_data[i][1]);

        return true;
    }

    bool selfTest() override { return true; }

    bool onSimpleUpdate()
    {
        lsmData_t raw_data;
        uint8_t buf[20];

        // Read temp, gyro, accel
        Bus::read(RegMap::OUT_TEMP_L, buf, 14);
        memcpy(&raw_data.buf, buf, 14);

        mLastTemp = normalizeTemp(raw_data.temp);

        mLastGyro.setX(normalizeGyro(raw_data.gyro[0]));
        mLastGyro.setY(normalizeGyro(raw_data.gyro[1]));
        mLastGyro.setZ(normalizeGyro(raw_data.gyro[2]));

        mLastAccel.setX(normalizeAccel(raw_data.accel[0]));
        mLastAccel.setY(normalizeAccel(raw_data.accel[1]));
        mLastAccel.setZ(normalizeAccel(raw_data.accel[2]));

        return true;
    }

    // clang-format off
    enum gyroFullScale
    {
        GYRO_FS_250           = 0,
        GYRO_FS_500           = 1,
        GYRO_FS_1000          = 2,
        GYRO_FS_2000          = 3
    };

    enum accelFullScale
    {
        ACC_FS_2G             = 0, // Do not change this sequence
        ACC_FS_4G             = 2,
        ACC_FS_8G             = 3,
        ACC_FS_16G            = 1
    };
    // clang-format on

private:
    constexpr static uint8_t whoami_value = 0x69;
    constexpr static float accelFSMAP[4]  = {0.061, 0.488, 0.122, 0.244};
    constexpr static float gyroFSMAP[4]   = {8.75, 17.50, 35.0, 70.0};
    uint8_t accelFS;
    uint16_t gyroFS;

    inline float normalizeAccel(int16_t val)
    {
        return static_cast<float>(val) / 1000.0f * accelFSMAP[accelFS] *
               EARTH_GRAVITY;  // [m/ss]
    }

    inline float normalizeGyro(int16_t val)
    {
        return static_cast<float>(val) / 1000.0f * gyroFSMAP[gyroFS] *
               DEGREES_TO_RADIANS;  // [rad/s]
    }

    inline float normalizeTemp(int16_t val)
    {
        return static_cast<float>(val) / 16.0f + 25.0f;  // [deg C]
    }

    enum RegMap
    {
        WHO_AM_I = 0x0F,  // default value

        // Accelerometer and gyroscope control registers
        CTRL1_XL = 0x10,
        CTRL2_G  = 0x11,
        CTRL3_C  = 0x12,
        CTRL4_C  = 0x13,
        CTRL5_C  = 0x14,
        CTRL6_C  = 0x15,
        CTRL7_G  = 0x16,
        CTRL8_XL = 0x17,
        CTRL9_XL = 0x18,
        CTRL10_C = 0x19,

        // Temperature output registers
        OUT_TEMP_L = 0x20,
        OUT_TEMP_H = 0x21,

        // Gyro data registers
        OUT_X_L_G = 0x22,
        OUT_X_H_G = 0x23,
        OUT_Y_L_G = 0x24,
        OUT_Y_H_G = 0x25,
        OUT_Z_L_G = 0x26,
        OUT_Z_H_G = 0x27,

        // Accelerometer output registers
        OUT_X_L_XL = 0x28,
        OUT_X_H_XL = 0x29,
        OUT_Y_L_XL = 0x2A,
        OUT_Y_H_XL = 0x2B,
        OUT_Z_L_XL = 0x2C,
        OUT_Z_H_XL = 0x2D,

        // STATUS register
        STATUS_REG = 0X1E
    };
};

template <typename Bus>
constexpr float LSM6DS3H<Bus>::accelFSMAP[];

template <typename Bus>
constexpr float LSM6DS3H<Bus>::gyroFSMAP[];

#endif /* ifndef LSM6DS3H */
