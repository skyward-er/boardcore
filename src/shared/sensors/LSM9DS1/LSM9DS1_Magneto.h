/* LSM9DS1 magnetometer Driver
 *
 * Copyright (c) 2016,2020 Skyward Experimental Rocketry
 * Authors: Andrea Milluzzo
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

#pragma once
#include <miosix.h>

#include <vector>

#include "LSM9DS1_Data.h"
#include "drivers/spi/SPIDriver.h"
#include "math/Stats.h"

using miosix::GpioPin;
using std::vector;

/**
 * @brief LSM9DS1 magnetometer sensor driver.
 * provides access to data generated by the sensor using SPI protocol.
 */

class LSM9DS1_M : public CompassSensor
{
public:
    enum MagFSR : uint8_t
    {
        FS_4  = 0x00,  // +/- 4Gauss
        FS_8  = 0x01,  // +/- 8Gauss
        FS_12 = 0x02,  // +/- 12Gauss
        FS_16 = 0x03   // +/- 16Gauss
    };

    enum ODR : uint8_t
    {
        ODR_0_625 = 0X00,  // 0.625Hz
        ODR_1_25  = 0x01,  // 1.25Hz
        ODR_2_5   = 0x02,  // 2.5Hz
        ODR_5     = 0x03,  // 5Hz
        ODR_10    = 0x04,  // 10Hz
        ODR_20    = 0x05,  // 20Hz
        ODR_40    = 0x06,  // 40Hz
        ODR_80    = 0x07   // 80Hz
    };

    // Sesitivity Map (axelFSR)
    const std::map<MagFSR, float> magFSR_SMap{{FS_4, 0.00014f},
                                              {FS_8, 0.00029f},
                                              {FS_12, 0.00043f},
                                              {FS_16, 0.00058f}};

    /**
     * @brief Creates an instance of an LSM9DS1 magnetometer sensor.
     *
     * @param    bus SPI bus the sensor is connected to
     * @param    cs Chip Select GPIO
     * @param    magRange magnetometer Full Scale Range (See datasheet)
     * @param    odr Output Data Rate (See datasheet)
     */

    LSM9DS1_M(SPIBusInterface& bus, GpioPin cs, MagFSR magRange = MagFSR::FS_16,
              ODR odr = ODR::ODR_80);

    /**
     * @brief Creates an instance of an LSM9DS1 magnetometer sensor.
     *
     * @param    bus SPI bus the sensor is connected to
     * @param    cs Chip Select GPIO
     * @param    config custom SPIBusConfig
     * @param    magRange magnetometer Full Scale Range (See datasheet)
     * @param    odr Output Data Rate (See datasheet)
     */

    LSM9DS1_M(SPIBusInterface& bus, GpioPin cs, SPIBusConfig config,
              MagFSR magRange = MagFSR::FS_16, ODR odr = ODR::ODR_80);

    /**
     * @brief initializes the LSM9DS1 Sensor (Magnetometer).
     * @return true if all setup registers of the sensor have been written
     * correctly.false if already initialized, wrong who_am_i or uncorrect
     * write.
     */

    bool init() override;

    /**
     * @brief Run a self-test of the Sensor.
     * @return true if sensor behave correclty
     */

    bool selfTest() override;

    /**
     * @brief Dump single reading of Magneto from the sensor through SPI.
     * @return true if sensor sends data
     */

    bool onSimpleUpdate() override;

private:
    void getSelfTestData(Stats& outx, Stats& outy, Stats& outz);

    bool sensor_initialized = false;
    bool selfTest_mode = false;

    SPISlave spislave;

    MagFSR magFSR;
    ODR odr;

    lsm9ds1MSample lastMagneto; 

    /**
     * @brief Registers' addresses definition.
     */
    enum regMapM
    {
        OFFSET_X_REG_L_M = 0x05,
        OFFSET_X_REG_H_M = 0x06,
        OFFSET_Y_REG_L_M = 0x07,
        OFFSET_Y_REG_H_M = 0x08,
        OFFSET_Z_REG_L_M = 0x09,
        OFFSET_Z_REG_H_M = 0x0A,
        WHO_AM_I_M       = 0x0F,
        CTRL_REG1_M      = 0x20,
        CTRL_REG2_M      = 0x21,
        CTRL_REG3_M      = 0x22,
        CTRL_REG4_M      = 0x23,
        CTRL_REG5_M      = 0x24,
        STATUS_REG_M     = 0x27,
        OUT_X_L_M        = 0x28,
        OUT_X_H_M        = 0x29,
        OUT_Y_L_M        = 0x2A,
        OUT_Y_H_M        = 0x2B,
        OUT_Z_L_M        = 0x2C,
        OUT_Z_H_M        = 0x2D,
        INT_CFG_M        = 0x30,
        INT_SRC_M        = 0x31,
        INT_THS_L_M      = 0x32,
        INT_THS_H_M      = 0x33
    };

    static const uint8_t WHO_AM_I_M_VAL  = 0x3D;
    static const uint8_t CTRL_REG3_M_VAL = 0x80;
    static const uint8_t CTRL_REG4_M_VAL = 0x0C;
    static const uint8_t INT_CFG_M_VAL   = 0x00;

    static const uint8_t AUTO_INCREMENT_ADDR = 0x40;
    static const uint8_t SOFT_RESET          = 0x08;

    // For selfTEST - limit for FS = +/-12Gauss
    // LIS3MDL limits - seems to be same magnetometer as LSM9DS1
    static constexpr float ST_XY_MIN = 1.0f;
    static constexpr float ST_XY_MAX = 3.0f;
    static constexpr float ST_Z_MIN  = 0.1f;
    static constexpr float ST_Z_MAX  = 1.0f;

    static const uint8_t DRDY_MASK            = 0x04;
    static const uint8_t SELFTEST_MAX_SAMPLES = 10;
    static const uint8_t SELFTEST_ENABLE      = 0x01;
};