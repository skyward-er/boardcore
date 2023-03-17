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
#include <utils/Debug.h>

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

    constexpr int NUM_SAMPLES = 5;
    constexpr int SLEEP_TIME  = 50;

    // Absolute value of extra tolerance
    constexpr float t = 0.1f;

    // Range which delta must be between, one for axis and expressed as {min,
    // max}. The unit is gauss.
    constexpr float ST_min = 0.015;
    constexpr float ST_max = 0.500;

    float avgX = 0.f, avgY = 0.f, avgZ = 0.f;

    // Set configuration for selfTest procedure. selfTest still not enabled
    {
        SPITransaction spi(slave);
        spi.writeRegister(CFG_REG_A,
                          ENABLE_TEMPERATURE_COMP | ODR_100_HZ | MD_CONTINUOUS);
        spi.writeRegister(CFG_REG_B,
                          spi.readRegister(CFG_REG_B) | OFFSET_CANCELLATION);
        spi.writeRegister(CFG_REG_C, spi.readRegister(CFG_REG_C) | ENABLE_BDU);
    }
    miosix::Thread::sleep(20);

    for (int i = 0; i < NUM_SAMPLES; ++i)
    {
        miosix::Thread::sleep(SLEEP_TIME);

        LIS2MDLData lastData = sampleImpl();
        avgX += lastData.magneticFieldX;
        avgY += lastData.magneticFieldY;
        avgZ += lastData.magneticFieldZ;
    }

    avgX /= NUM_SAMPLES;
    avgY /= NUM_SAMPLES;
    avgZ /= NUM_SAMPLES;

    {
        // selfTest is enabled
        SPITransaction spi(slave);
        spi.writeRegister(CFG_REG_C,
                          spi.readRegister(CFG_REG_C) | ENABLE_SELF_TEST);
    }
    miosix::Thread::sleep(60);

    // Deltas: absolute difference between the values measured before and after
    float deltas[3];

    LIS2MDLData lastData = sampleImpl();
    deltas[0]            = std::abs(lastData.magneticFieldX - avgX);
    deltas[1]            = std::abs(lastData.magneticFieldY - avgY);
    deltas[2]            = std::abs(lastData.magneticFieldZ - avgZ);

    bool passed = true;
    for (int j = 0; j < 3; ++j)
        if (deltas[j] < (ST_max - t) && deltas[j] > (ST_min + t))
            passed = false;

    // Reset configuration, then return
    applyConfig(configuration);

    if (!passed)
    {
        lastError = SELF_TEST_FAIL;
        return false;
    }

    {
        // Disable selfTest
        SPITransaction spi(slave);
        spi.writeRegister(CFG_REG_C,
                          spi.readRegister(CFG_REG_C) & ~ENABLE_SELF_TEST);
    }

    return true;
}

bool LIS2MDL::applyConfig(Config config)
{
    SPITransaction spi(slave);
    uint8_t reg = 0;

    // CFG_REG_A: configuration register
    reg |= config.odr << 2;
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