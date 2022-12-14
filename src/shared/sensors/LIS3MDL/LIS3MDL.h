/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Author: Riccardo Musso
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include <diagnostic/PrintLogger.h>
#include <drivers/spi/SPIDriver.h>
#include <sensors/Sensor.h>

#include "LIS3MDLData.h"

namespace Boardcore
{

/**
 * Driver for LIS3MDL, a three-axis magnetic sensor.
 */
class LIS3MDL : public Sensor<LIS3MDLData>
{
public:
    /**
     * @brief Constants for Output Data Rate configuration.
     *
     * Note: constants values already include axis operative mode selection and
     * FAST_ODR options, so they are ready to be put inside CTRL_REG1 along with
     * TEMP_EN and ST.
     */
    enum ODR : uint8_t
    {
        ODR_0_625_HZ = 0x00,  //!< 0.625 Hz
        ODR_1_25_HZ  = 0x04,  //!<  1.25 Hz
        ODR_2_5_HZ   = 0x08,  //!<   2.5 Hz
        ODR_5_HZ     = 0x0c,  //!<     5 Hz
        ODR_10_HZ    = 0x10,  //!<    10 Hz
        ODR_20_HZ    = 0x14,  //!<    20 Hz
        ODR_40_HZ    = 0x18,  //!<    40 Hz
        ODR_80_HZ    = 0x1c,  //!<    80 Hz

        ODR_155_HZ  = 0x62,  //!<    155 Hz
        ODR_300_HZ  = 0x42,  //!<    300 Hz
        ODR_560_HZ  = 0x22,  //!<    560 Hz
        ODR_1000_HZ = 0x02,  //!<   1000 Hz

        /**
         * Constant used by the driver: this bit
         * is 1 for ODR > 80 Hz, set to 0 otherwise.
         */
        FAST_ODR_BIT = 0x02,
    };

    enum FullScale : uint8_t
    {
        FS_4_GAUSS  = 0x00,  ///<  +/- 4  gauss
        FS_8_GAUSS  = 0x20,  ///<  +/- 8  gauss
        FS_12_GAUSS = 0x40,  ///<  +/- 12 gauss
        FS_16_GAUSS = 0x60,  ///<  +/- 16 gauss
    };

    /**
     * @brief Operative mode constants.
     *
     * Operative mode (that is power consumption) options. Higher power implies
     * better performance.
     */
    enum OperativeMode : uint8_t
    {
        OM_LOW_POWER        = 0x0,
        OM_MEDIUM_POWER     = 0x1,
        OM_HIGH_POWER       = 0x2,
        OM_ULTRA_HIGH_POWER = 0x3,
    };

    /**
     * @brief Sensor configuration
     *
     * This struct contains all the settings the user
     * is able to modify with the help of the driver.
     * They are applied in the constructor of LIS3MDL class
     * and on each call of LIS3MDL::applyConfig()
     */
    struct Config
    {
        Config() {}

        /**
         * @brief Full scale setting.
         *
         * @see LIS3MDL::FullScale
         */
        FullScale scale = FS_8_GAUSS;

        /**
         * @brief Data rate configuration
         *
         * Default: 40 Hz
         *
         * Important: if ODR is set more than 80 Hz, operative mode of x and y
         * axis will be set accordingly.
         *
         * @see LIS3MDL::ODR
         */
        ODR odr = ODR_40_HZ;

        /**
         * @brief Operative mode for x and y axis.
         *
         * Note: if ODR is greater than 80 Hz, this setting will be ignored and
         * actual operative mode will be set depending of the chosen frequency.
         *
         * @see LIS3MDL::OperativeMode
         */
        OperativeMode xyMode = OM_ULTRA_HIGH_POWER;

        /**
         * Operative mode for z axis.
         *
         * @see LIS3MDL::OM
         */
        OperativeMode zMode = OM_ULTRA_HIGH_POWER;

        /**
         * Enables temperature sensor.
         * Default: true
         */
        bool enableTemperature = true;

        /**
         * @brief Sets the value of tempDivider.
         *
         * With the given value you can instruct the driver to update
         * the temperature according to a different rate.
         * The temperature will be updated only once in `tempDivider` calls
         * to sampleImpl(), so for example:
         * 2 -> updated half the times,
         * 1 -> updated every time.
         */
        unsigned temperatureDivider = 1;

        /**
         * @brief Enables interrupts
         *
         * Whether are interrupts enabled respectively on the x, y and z axis.
         * If it is set to true on at least one axis, the interrupts will be
         * generated otherwise, they will be completely disabled by the driver.
         */
        bool enableInterrupt[3] = {false, false, false};

        /**
         * Absolute value of the threshold that triggers the interrupt
         * (expressed in gauss).
         */
        float threshold = 0;

        /**
         * @brief BDU setting
         *
         * If set to true, the sensor won't update the data until it is read, if
         * false the sensor data will be continuously updated.
         */
        bool doBlockDataUpdate = false;
    };

    LIS3MDL(SPIBusInterface& bus, miosix::GpioPin pin,
            SPIBusConfig spiConfig = {}, Config config = {});

    bool init() override;

    bool selfTest() override;

    /**
     * @brief Overwrites the sensor settings.
     *
     * Writes a certain config to the sensor registers. This method is
     * automatically called in LIS3MDL::init() using as parameter the
     * configuration given in the constructor.
     *
     * This method checks if the values were actually written in the sensor's
     * registers and returns false if at least one of them was not as expected.
     *
     * @param config The configuration to be applied.
     * @returns True if the configuration was applied successfully,
     * false otherwise.
     */
    bool applyConfig(Config config);

private:
    LIS3MDLData sampleImpl() override;

    void updateUnit(FullScale fs);

    SPISlave mSlave;
    Config mConfig;

    unsigned currDiv;
    bool isInitialized;
    float mUnit = 0;

    enum Registers : uint8_t
    {
        WHO_AM_I = 0x0f,

        CTRL_REG1 = 0x20,
        CTRL_REG2 = 0x21,
        CTRL_REG3 = 0x22,
        CTRL_REG4 = 0x23,
        CTRL_REG5 = 0x24,

        STATUS_REG = 0x27,
        OUT_X_L    = 0x28,
        OUT_X_H    = 0x29,
        OUT_Y_L    = 0x2a,
        OUT_Y_H    = 0x2b,
        OUT_Z_L    = 0x2c,
        OUT_Z_H    = 0x2d,

        TEMP_OUT_L = 0x2e,
        TEMP_OUT_H = 0x2f,

        INT_CFG   = 0x30,
        INT_THS_L = 0x32,
        INT_THS_H = 0x33,
    };

    enum Constants : unsigned
    {
        WHO_AM_I_VALUE = 0x3d,

        CONTINUOS_CONVERSION  = 0x0,
        REFERENCE_TEMPERATURE = 25,
        LSB_PER_CELSIUS       = 8,

        LSB_PER_GAUSS_FS_4  = 6842,
        LSB_PER_GAUSS_FS_8  = 3421,
        LSB_PER_GAUSS_FS_12 = 2281,
        LSB_PER_GAUSS_FS_16 = 1711,

        ENABLE_TEMPERATURE = 0x80,
        ENABLE_SELF_TEST   = 0x01,
        ENABLE_BDU         = 0x40,

        ENABLE_INT_PIN = 0x01,
        ENABLE_INT_X   = 0x80,
        ENABLE_INT_Y   = 0x40,
        ENABLE_INT_Z   = 0x20,
    };

    PrintLogger logger = Logging::getLogger("lis3mdl");
};

}  // namespace Boardcore
