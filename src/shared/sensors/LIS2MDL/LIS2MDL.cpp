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

#include "LIS2MDL.h"

#include <drivers/timer/TimestampTimer.h>
#include <miosix.h>
#include <sensors/calibration/SensorDataExtra/SensorDataExtra.h>
#include <utils/Debug.h>

#include <iostream>

using namespace Eigen;

namespace Boardcore
{

LIS2MDL::LIS2MDL(SPIBusInterface& bus, miosix::GpioPin pin,
                 SPIBusConfig spiConfig, Config config)
    : slave(bus, pin, spiConfig), configuration(config)
{
}

SPIBusConfig LIS2MDL::getDefaultSPIConfig()
{
    SPIBusConfig spiConfig;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_256;
    spiConfig.mode         = SPI::Mode::MODE_3;
    spiConfig.byteOrder    = SPI::Order::LSB_FIRST;
    return spiConfig;
}

bool LIS2MDL::init()
{
    if (isInitialized)
    {
        LOG_ERR(logger, "Attempted to initialized sensor twice but failed");
        lastError = ALREADY_INIT;
        return false;
    }

    {
        SPITransaction spi(slave);
        spi.writeRegister(CFG_REG_C, ENABLE_4WSPI | I2C_DISABLE);
    }

    {
        SPITransaction spi(slave);
        uint8_t res = spi.readRegister(WHO_AM_I);

        if (res != WHO_AM_I_VALUE)
        {
            LOG_ERR(logger,
                    "WHO_AM_I value differs from expectation: read 0x{:x} "
                    "but expected 0x{:x}",
                    res, WHO_AM_I_VALUE);
            lastError = INVALID_WHOAMI;
            return false;
        }
    }

    isInitialized = true;
    return applyConfig(configuration);
}

bool LIS2MDL::selfTest()
{
    if (!isInitialized)
    {
        LOG_ERR(logger, "Invoked selfTest() but sensor was uninitialized");
        lastError = NOT_INIT;
        return false;
    }

    /**
     * The device has to be kept still while the self-test is ongoing.
     * See AN5069 for further details on the self-test procedure.
     */

    static constexpr int NUM_SAMPLES = 50;  // 50 samples suggested by AN5069
    static constexpr int SLEEP_TIME  = 10;  // 100Hz -> 10ms between samples
    Vector3f avgPreTest              = Vector3f::Zero();
    Vector3f avgPostTest             = Vector3f::Zero();
    Vector3f tmp;

    // 1. Set configuration for selfTest procedure. selfTest still not enabled
    {
        SPITransaction spi(slave);
        spi.writeRegister(CFG_REG_A,
                          ENABLE_TEMPERATURE_COMP | ODR_100_HZ | MD_CONTINUOUS);
        spi.writeRegister(CFG_REG_B,
                          spi.readRegister(CFG_REG_B) | OFFSET_CANCELLATION);
        spi.writeRegister(CFG_REG_C, spi.readRegister(CFG_REG_C) | ENABLE_BDU);
    }

    // Wait for power up, ~20ms for a stable output
    miosix::Thread::sleep(20);

    // 2. Averaging fifty samples before enabling the self-test
    {
        for (int i = 0; i < NUM_SAMPLES - 1; i++)
        {
            tmp << static_cast<MagnetometerData>(sampleImpl());
            avgPreTest += tmp;

            miosix::Thread::sleep(SLEEP_TIME);
        }
        tmp << static_cast<MagnetometerData>(sampleImpl());
        avgPreTest += tmp;

        // Compute average
        avgPreTest /= NUM_SAMPLES;
    }

    // 3. Enable self-test
    {
        SPITransaction spi(slave);
        spi.writeRegister(CFG_REG_C,
                          spi.readRegister(CFG_REG_C) | ENABLE_SELF_TEST);
    }

    // Wait 60ms (suggested in AN)
    miosix::Thread::sleep(60);

    // 4. Averaging fifty samples after enabling the self-test
    {
        for (int i = 0; i < NUM_SAMPLES - 1; i++)
        {
            tmp << static_cast<MagnetometerData>(sampleImpl());
            avgPostTest += tmp;

            miosix::Thread::sleep(SLEEP_TIME);
        }
        tmp << static_cast<MagnetometerData>(sampleImpl());
        avgPostTest += tmp;

        // Compute average
        avgPostTest /= NUM_SAMPLES;
    }

    // 5. Computing the difference in the module for each axis and verifying
    //    that is falls in the given range: the min and max value are provided
    //    in the datasheet.
    {
        Vector3f deltas = (avgPostTest - avgPreTest).cwiseAbs();

        // Range which delta must be between, one for axis and expressed as
        // {min, max}. The unit is gauss.
        static constexpr float ST_MIN = 0.015;  // [Gauss]
        static constexpr float ST_MAX = 0.500;  // [Gauss]

        bool passed =
            (ST_MIN < deltas.array()).all() && (deltas.array() < ST_MAX).all();

        // Reset configuration, then return
        applyConfig(configuration);

        if (!passed)
        {
            LOG_ERR(logger, "selfTest() failed");
            lastError = SELF_TEST_FAIL;
            return false;
        }
        else
        {
            return true;
        }
    }
}

bool LIS2MDL::applyConfig(Config config)
{
    SPITransaction spi(slave);
    uint8_t reg = 0;

    // CFG_REG_A: configuration register
    reg |= config.odr;
    reg |= config.deviceMode;
    reg |= (1 << 7);
    reg |= (spi.readRegister(CFG_REG_A) & 0b01110000);
    spi.writeRegister(CFG_REG_A, reg);

    return true;
}

LIS2MDLData LIS2MDL::sampleImpl()
{
    if (!isInitialized)
    {
        LOG_ERR(logger, "Invoked sampleImpl() but sensor was uninitialized");
        lastError = NOT_INIT;
        return lastSample;
    }

    SPITransaction spi(slave);
    LIS2MDLData newData;

    // Check if the temperature has to be sampled
    tempCounter++;
    if (configuration.temperatureDivider != 0 &&
        tempCounter % configuration.temperatureDivider == 0)
    {
        int16_t outTemp              = spi.readRegister16(TEMP_OUT_L_REG);
        newData.temperatureTimestamp = TimestampTimer::getTimestamp();
        newData.temperature          = DEG_PER_LSB * outTemp;
        newData.temperature += REFERENCE_TEMPERATURE;
    }
    else
    {
        newData.temperature = lastSample.temperature;
    }

    uint8_t values[6];
    spi.readRegisters(OUTX_L_REG, values, sizeof(values));

    int16_t outX = values[1] << 8 | values[0];
    int16_t outY = values[3] << 8 | values[2];
    int16_t outZ = values[5] << 8 | values[4];

    newData.magneticFieldTimestamp = TimestampTimer::getTimestamp();
    newData.magneticFieldX         = GAUSS_PER_LSB * outX;
    newData.magneticFieldY         = GAUSS_PER_LSB * outY;
    newData.magneticFieldZ         = GAUSS_PER_LSB * outZ;

    return newData;
}

}  // namespace Boardcore