/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Giulia Ghirardini
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

#include "LIS2MDLData.h"

namespace Boardcore
{

/**
 * Driver for LIS2MDL, a three-axis magnetic sensor.
 */
class LIS2MDL : public Sensor<LIS2MDLData>
{
public:
    /**
     * @brief Constants for Output Data Rate configuration.
     *
     */
    enum ODR : uint8_t
    {
        ODR_10_HZ  = 0x00,  //!< 10 Hz
        ODR_20_HZ  = 0x01,  //!< 20 Hz
        ODR_50_HZ  = 0x02,  //!< 50 Hz
        ODR_100_HZ = 0x03,  //!< 100 Hz
    };

    /**
     * @brief Mode of operation constants.
     *
     */
    enum OperativeMode : uint8_t
    {
        MD_CONTINUOUS = 0x00,
        MD_SINGLE     = 0x01,
        MD_IDLE0      = 0x02,
        MD_IDLE1      = 0x03,
    };

    /**
     * @brief Sensor configuration
     *
     * This struct contains all the settings the user
     * is able to modify with the help of the driver.
     * They are applied in the constructor of LIS2MDL class
     * and on each call of LIS2MDL::applyConfig()
     */
    struct Config
    {
        Config() {}
        /**
         * @brief Data rate configuration
         *
         * Default: 10 Hz
         *
         * @see LIS2MDL::ODR
         */
        ODR odr = ODR_10_HZ;

        /**
         * @brief Mode of operation of the device
         * Default value: 11 - Idle mode 2
         *
         * @see LIS2MDL::OperativeMode
         */
        OperativeMode deviceMode = MD_IDLE1;

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

    LIS2MDL(SPIBusInterface& bus, miosix::GpioPin pin,
            SPIBusConfig spiConfig = {}, Config config = {});

    bool init() override;

    bool selfTest() override;

    /**
     * @brief Overwrites the sensor settings.
     *
     * Writes a certain config to the sensor registers. This method is
     * automatically called in LIS2MDL::init() using as parameter the
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
    LIS2MDLData sampleImpl() override;

    SPISlave mSlave;
    Config mConfig;

    unsigned currDiv;
    bool isInitialized;
    float mUnit = 0;

    enum Registers : uint8_t
    {
        OFFSET_X_REG_L = 0x45,
        OFFSET_X_REG_H = 0x46,
        OFFSET_Y_REG_L = 0x47,
        OFFSET_Y_REG_H = 0x48,
        OFFSET_Z_REG_L = 0x49,
        OFFSET_Z_REG_H = 0x4a,

        WHO_AM_I = 0x4f,

        CFG_REG_A = 0x60,
        CFG_REG_B = 0x61,
        CFG_REG_C = 0x62,

        INT_CRTL_REG   = 0x63,
        INT_SOURCE_REG = 0x64,
        INT_THS_L_REG  = 0x65,
        INT_THS_H_REG  = 0x66,
        STATUS_REG     = 0x67,

        OUTX_L_REG = 0x68,
        OUTX_H_REG = 0x69,
        OUTY_L_REG = 0x6a,
        OUTY_H_REG = 0x6b,
        OUTZ_L_REG = 0x6c,
        OUTZ_H_REG = 0x6d,

        TEMP_OUT_L_REG = 0x6e,
        TEMP_OUT_H_REG = 0x6f,
    };

    static constexpr uint32_t WHO_AM_I_VALUE       = 0x40;
    static constexpr uint32_t CONTINUOS_CONVERSION = 0x0;

    static constexpr float REFERENCE_TEMPERATURE = 25;
    static constexpr float LSB_PER_CELSIUS       = 8;
    static constexpr float LSB_PER_GAUSS_MIN     = 0.001395;
    static constexpr float LSB_PER_GAUSS_MAX     = 0.001605;

    static constexpr uint32_t ENABLE_TEMPERATURE = 0x80;
    static constexpr uint32_t ENABLE_SELF_TEST   = 0x02;
    static constexpr uint32_t ENABLE_BDU         = 0x10;
    static constexpr uint32_t ENABLE_INT_PIN     = 0x01;
    static constexpr uint32_t ENABLE_INT_X       = 0x80;
    static constexpr uint32_t ENABLE_INT_Y       = 0x40;
    static constexpr uint32_t ENABLE_INT_Z       = 0x20;

    PrintLogger logger = Logging::getLogger("lis2mdl");
};

}  // namespace Boardcore
