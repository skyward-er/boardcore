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

#include "LIS3MDL.h"

#include <drivers/timer/TimestampTimer.h>

namespace Boardcore
{

LIS3MDL::LIS3MDL(SPIBusInterface& bus, miosix::GpioPin pin,
                 SPIBusConfig spiConfig, Config config)
    : slave(bus, pin, spiConfig), configuration(config)
{
}

bool LIS3MDL::init()
{
    if (isInitialized)
    {
        LOG_ERR(logger, "Attempted to initialized sensor twice but failed");
        lastError = ALREADY_INIT;
        return false;
    }

    {
        SPITransaction spi(slave);
        uint8_t res = spi.readRegister(WHO_AM_I);

        if (res != WHO_AM_I_VALUE)
        {
            LOG_ERR(logger,
                    "WHO_AM_I value differs from expectation: read 0x{02x} "
                    "but expected 0x{02x}",
                    res, WHO_AM_I_VALUE);
            lastError = INVALID_WHOAMI;
            return false;
        }
    }

    isInitialized = true;
    return applyConfig(configuration);
}

bool LIS3MDL::selfTest()
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
    constexpr float deltaRange[3][2] = {{1.f, 3.f}, {1.f, 3.f}, {0.1f, 1.f}};

    float avgX = 0.f, avgY = 0.f, avgZ = 0.f;

    {
        SPITransaction spi(slave);
        spi.writeRegister(CTRL_REG2, FS_12_GAUSS);
    }
    updateUnit(FS_12_GAUSS);

    for (int i = 0; i < NUM_SAMPLES; ++i)
    {
        miosix::Thread::sleep(SLEEP_TIME);

        LIS3MDLData lastData = sampleImpl();
        avgX += lastData.magneticFieldX;
        avgY += lastData.magneticFieldY;
        avgZ += lastData.magneticFieldZ;
    }

    avgX /= NUM_SAMPLES;
    avgY /= NUM_SAMPLES;
    avgZ /= NUM_SAMPLES;

    // Setting up the sensor settings for proper usage of the self test mode.
    {
        SPITransaction spi(slave);

        spi.writeRegister(CTRL_REG1, ODR_20_HZ | ENABLE_SELF_TEST |
                                         (OM_ULTRA_HIGH_POWER << 4));
        spi.writeRegister(CTRL_REG2, FS_12_GAUSS);
        spi.writeRegister(CTRL_REG4, OM_ULTRA_HIGH_POWER << 2);
    }

    // Deltas: absolute difference between the values measured before and after
    float deltas[3];

    miosix::Thread::sleep(SLEEP_TIME);

    LIS3MDLData lastData = sampleImpl();
    deltas[0]            = std::abs(lastData.magneticFieldX - avgX);
    deltas[1]            = std::abs(lastData.magneticFieldY - avgY);
    deltas[2]            = std::abs(lastData.magneticFieldZ - avgZ);

    bool passed = true;
    for (int j = 0; j < 3; ++j)
        if (deltas[j] < (deltaRange[j][0] - t) ||
            deltas[j] > (deltaRange[j][1] + t))
            passed = false;

    // Reset configuration, then return
    applyConfig(configuration);

    if (!passed)
    {
        lastError = SELF_TEST_FAIL;
        return false;
    }

    return applyConfig(configuration);
}

bool LIS3MDL::applyConfig(Config config)
{

    SPITransaction spi(slave);
    uint8_t reg = 0, err = 0;

    configuration = config;

    // CTRL_REG1
    if (config.temperatureDivider != 0)
    {
        reg = ENABLE_TEMPERATURE;
    }
    reg |= config.odr;

    // odr <= 80Hz
    if (!(config.odr & FAST_ODR_BIT))
        reg |= config.xyMode << 4;
    spi.writeRegister(CTRL_REG1, reg);
    err |= spi.readRegister(CTRL_REG1) != reg;

    // CTRL_REG2
    reg = config.scale;
    spi.writeRegister(CTRL_REG2, reg);
    err |= spi.readRegister(CTRL_REG2) != reg;

    // CTRL_REG3
    reg = CONTINUOS_CONVERSION;
    spi.writeRegister(CTRL_REG3, reg);
    err |= spi.readRegister(CTRL_REG3) != reg;

    // CTRL_REG4
    reg = config.zMode << 2;
    spi.writeRegister(CTRL_REG4, reg);
    err |= spi.readRegister(CTRL_REG4) != reg;

    // CTRL_REG5
    if (config.doBlockDataUpdate)
        reg = ENABLE_BDU;
    else
        reg = 0;

    spi.writeRegister(CTRL_REG5, reg);
    err |= spi.readRegister(CTRL_REG5) != reg;

    // Set mUnit according to scale
    updateUnit(config.scale);

    if (err)
    {
        LOG_ERR(logger, "Spi error");
        lastError = BUS_FAULT;
        return false;
    }

    return true;
}

LIS3MDLData LIS3MDL::sampleImpl()
{
    if (!isInitialized)
    {
        LOG_ERR(logger, "Invoked sampleImpl() but sensor was uninitialized");
        lastError = NOT_INIT;
        return lastSample;
    }

    SPITransaction spi(slave);
    LIS3MDLData newData;

    tempCounter++;
    if (configuration.temperatureDivider != 0 &&
        tempCounter % configuration.temperatureDivider == 0)
    {
        uint8_t values[2];
        spi.readRegisters(TEMP_OUT_L, values, sizeof(values));

        int16_t outTemp              = values[1] << 8 | values[0];
        newData.temperatureTimestamp = TimestampTimer::getTimestamp();
        newData.temperature          = DEG_PER_LSB * outTemp;
        newData.temperature += REFERENCE_TEMPERATURE;
    }
    else
    {
        newData.temperature = lastSample.temperature;
    }

    uint8_t values[6];
    spi.readRegisters(OUT_X_L, values, sizeof(values));

    int16_t outX = values[1] << 8 | values[0];
    int16_t outY = values[3] << 8 | values[2];
    int16_t outZ = values[5] << 8 | values[4];

    newData.magneticFieldTimestamp = TimestampTimer::getTimestamp();
    newData.magneticFieldX         = currentUnit * outX;
    newData.magneticFieldY         = currentUnit * outY;
    newData.magneticFieldZ         = currentUnit * outZ;

    return newData;
}

void LIS3MDL::updateUnit(FullScale fs)
{
    switch (fs)
    {
        case FS_4_GAUSS:
            currentUnit = GAUSS_PER_LSB_FS_4;
            break;

        case FS_8_GAUSS:
            currentUnit = GAUSS_PER_LSB_FS_8;
            break;

        case FS_12_GAUSS:
            currentUnit = GAUSS_PER_LSB_FS_12;
            break;

        case FS_16_GAUSS:
            currentUnit = GAUSS_PER_LSB_FS_16;
            break;
    }
};

}  // namespace Boardcore
