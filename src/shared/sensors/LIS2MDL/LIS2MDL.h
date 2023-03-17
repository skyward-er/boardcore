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
 * @brief Driver for LIS2MDL, a three-axis magnetic sensor.
 */
class LIS2MDL : public Sensor<LIS2MDLData>
{
public:
    enum ODR : uint8_t
    {
        ODR_10_HZ  = 0x00,  ///< 10 Hz
        ODR_20_HZ  = 0x01,  ///< 20 Hz
        ODR_50_HZ  = 0x02,  ///< 50 Hz
        ODR_100_HZ = 0x03,  ///< 100 Hz
    };

    enum OperativeMode : uint8_t
    {
        MD_CONTINUOUS = 0x00,
        MD_SINGLE     = 0x01,
        MD_IDLE0      = 0x02,
        MD_IDLE1      = 0x03,
    };

    /**
     * @brief Sensor configuration.
     *
     * This struct contains all the settings the user is able to modify with the
     * help of the driver. They are applied in the constructor of LIS2MDL class
     * and on each call of LIS2MDL::applyConfig()
     */
    struct Config
    {
        Config() {}

        ODR odr                  = ODR_10_HZ;
        OperativeMode deviceMode = MD_IDLE1;

        /**
         * @brief Divide the temperature sampling rate.
         *
         * This is used to limit the sampling of the temperature, use 0 to
         * disable it completely.
         */
        unsigned temperatureDivider = 0;
    };

    LIS2MDL(SPIBusInterface& bus, miosix::GpioPin pin,
            SPIBusConfig spiConfig = {}, Config config = {});

    static SPIBusConfig getDefaultSPIConfig();

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

    SPISlave slave;
    Config configuration;

    unsigned tempCounter = 0;
    bool isInitialized   = false;

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
    static constexpr float DEG_PER_LSB           = 0.125;

    static constexpr float GAUSS_PER_LSB = 0.0015;

    static constexpr uint32_t ENABLE_TEMPERATURE_COMP = 1 << 7;
    static constexpr uint32_t ENABLE_SELF_TEST        = 1 << 1;
    static constexpr uint32_t ENABLE_BDU              = 1 << 4;
    static constexpr uint32_t ENABLE_4WSPI            = 1 << 2;
    static constexpr uint32_t OFFSET_CANCELLATION     = 1 << 1;
    static constexpr uint32_t I2C_DISABLE             = 1 << 5;

    PrintLogger logger = Logging::getLogger("lis2mdl");
};

}  // namespace Boardcore
