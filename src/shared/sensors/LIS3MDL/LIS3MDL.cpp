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
    : mSlave(bus, pin, spiConfig), mConfig(config), currDiv(0),
      isInitialized(false)
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
        SPITransaction spi(mSlave);
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
    return applyConfig(mConfig);
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
        SPITransaction spi(mSlave);
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
        SPITransaction spi(mSlave);

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

    if (!passed)
    {
        // reset configuration, then return
        applyConfig(mConfig);

        lastError = SELF_TEST_FAIL;
        return false;
    }

    return applyConfig(mConfig);
}

bool LIS3MDL::applyConfig(Config config)
{

    SPITransaction spi(mSlave);
    uint8_t reg = 0, err = 0;

    mConfig = config;
    currDiv = 0;

    // CTRL_REG1
    if (config.enableTemperature)
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

    // INT_CFG
    if (config.enableInterrupt[0])
        reg = ENABLE_INT_X;
    else
        reg = 0;

    if (config.enableInterrupt[1])
        reg |= ENABLE_INT_Y;
    if (config.enableInterrupt[2])
        reg |= ENABLE_INT_Z;

    // The interrupt of at least one axis is enabled
    if (reg)
        reg |= ENABLE_INT_PIN;

    reg |= 0x08;
    spi.writeRegister(INT_CFG, reg);
    err |= spi.readRegister(INT_CFG) != reg;

    // INT_THS
    uint16_t val = static_cast<uint16_t>(std::abs(config.threshold / mUnit));
    reg          = static_cast<uint8_t>(0xff & val);
    spi.writeRegister(INT_THS_L, reg);
    err |= spi.readRegister(INT_THS_L) != reg;

    reg = static_cast<uint8_t>(val >> 8);
    reg &= 0x7f;  // Remove MSB (according to the datasheet, it must be zero)
    spi.writeRegister(INT_THS_H, reg);
    err |= spi.readRegister(INT_THS_H) != reg;

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

    SPITransaction spi(mSlave);

    if (!spi.readRegister(STATUS_REG))
    {
        lastError = NO_NEW_DATA;
        return lastSample;
    }

    // Reset any error
    lastError = SensorErrors::NO_ERRORS;

    int16_t val;
    LIS3MDLData newData{};

    if (mConfig.enableTemperature)
    {
        if (currDiv == 0)
        {
            val = spi.readRegister(TEMP_OUT_L);
            val |= spi.readRegister(TEMP_OUT_H) << 8;

            newData.temperatureTimestamp = TimestampTimer::getTimestamp();
            newData.temperature = static_cast<float>(val) / LSB_PER_CELSIUS +
                                  REFERENCE_TEMPERATURE;
        }
        else
        {
            // Keep old value
            newData.temperature = lastSample.temperature;
        }

        currDiv = (currDiv + 1) % mConfig.temperatureDivider;
    }

    newData.magneticFieldTimestamp = TimestampTimer::getTimestamp();

    val = spi.readRegister(OUT_X_L);
    val |= spi.readRegister(OUT_X_H) << 8;
    newData.magneticFieldX = mUnit * val;

    val = spi.readRegister(OUT_Y_L);
    val |= spi.readRegister(OUT_Y_H) << 8;
    newData.magneticFieldY = mUnit * val;

    val = spi.readRegister(OUT_Z_L);
    val |= spi.readRegister(OUT_Z_H) << 8;
    newData.magneticFieldZ = mUnit * val;

    return newData;
}

void LIS3MDL::updateUnit(FullScale fs)
{
    switch (fs)
    {
        case FS_4_GAUSS:
            mUnit = 1.f / LSB_PER_GAUSS_FS_4;
            break;

        case FS_8_GAUSS:
            mUnit = 1.f / LSB_PER_GAUSS_FS_8;
            break;

        case FS_12_GAUSS:
            mUnit = 1.f / LSB_PER_GAUSS_FS_12;
            break;

        case FS_16_GAUSS:
            mUnit = 1.f / LSB_PER_GAUSS_FS_16;
            break;
    }
};

}  // namespace Boardcore
