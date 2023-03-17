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
    : mSlave(bus, pin, spiConfig), mConfig(config), currDiv(0),
      isInitialized(false)
{
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
        // Important! It is imperative to get the 4WSPI enabled (set to the
        // value of 1) due to the four-wire connection for SPI and the I2C_DIS
        // disabled. selfTest bit still not enabled.
        SPITransaction spi(mSlave);
        spi.writeRegister(CFG_REG_C, (1 << 2) | (1 << 5));
    }

    {
        SPITransaction spi(mSlave);
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
    return applyConfig(mConfig);
}

bool LIS2MDL::selfTest()
{
    if (!isInitialized)
    {
        LOG_ERR(logger, "Invoked selfTest() but sensor was uninitialized");
        lastError = NOT_INIT;
        return false;
    }

    constexpr int NUM_SAMPLES = 50;
    constexpr int SLEEP_TIME  = 50;

    // Absolute value of extra tolerance
    constexpr float t = 0.1f;

    // Range which delta must be between, one for axis and expressed as {min,
    // max}. The unit is gauss.
    constexpr float deltaRange[3][2] = {{1.f, 3.f}, {1.f, 3.f}, {0.1f, 1.f}};

    float avgX = 0.f, avgY = 0.f, avgZ = 0.f;

    // Set configuration for selfTest procedure. selfTest still not enabled
    {
        SPITransaction spi(mSlave);
        // CFG_REG_A: 10001100
        // continuous mode, odr = 100 Hz, enable temperature compensation
        spi.writeRegister(CFG_REG_A, 140);
        // CFG_REG_B: 00000010
        // offset cancellation
        spi.writeRegister(CFG_REG_B, spi.readRegister(CFG_REG_B) | (1 << 1));
        // CFG_REG_C: 001(1)0110
        // BDU enabled. only add the value in parenthesis otherwise overwrite
        // data
        spi.writeRegister(CFG_REG_C, spi.readRegister(CFG_REG_C) | (1 << 4));
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
        SPITransaction spi(mSlave);
        spi.writeRegister(CFG_REG_C, spi.readRegister(CFG_REG_C) | (1 << 1));
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
        if (deltas[j] < (deltaRange[j][0] - t) &&
            deltas[j] > (deltaRange[j][1] + t))
            passed = false;

    // reset configuration, then return
    applyConfig(mConfig);

    if (!passed)
    {
        lastError = SELF_TEST_FAIL;
        return false;
    }

    {
        // Disable selfTest
        SPITransaction spi(mSlave);
        spi.writeRegister(CFG_REG_C, spi.readRegister(CFG_REG_C) & ~(1 << 1));
        // Set idle mode
        spi.writeRegister(CFG_REG_A,
                          spi.readRegister(CFG_REG_A) & ~((1 << 0) | (1 << 1)));
    }

    return true;
}

bool LIS2MDL::applyConfig(Config config)
{
    SPITransaction spi(mSlave);
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

    SPITransaction spi(mSlave);
    // check STATUS_REG to see if new data is available. If Zyxda = 1 (true) a
    // new set of data is available.
    if (!(spi.readRegister(STATUS_REG) | (1 << 4)))
    {
        lastError = NO_NEW_DATA;
        return lastSample;
    }

    // Reset any error
    lastError = SensorErrors::NO_ERRORS;

    int16_t val;
    LIS2MDLData newData{};

    if (mConfig.temperatureDivider != 0)
    {
        if (currDiv == 0)
        {
            val = spi.readRegister(TEMP_OUT_L_REG);
            val |= ((uint16_t)spi.readRegister(TEMP_OUT_H_REG)) << 8;

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

    val = spi.readRegister(OUTX_L_REG);
    val |= ((uint16_t)spi.readRegister(OUTX_H_REG)) << 8;
    newData.magneticFieldX = mUnit * val;

    val = spi.readRegister(OUTY_L_REG);
    val |= ((uint16_t)spi.readRegister(OUTY_H_REG)) << 8;
    newData.magneticFieldY = mUnit * val;

    val = spi.readRegister(OUTZ_L_REG);
    val |= ((uint16_t)spi.readRegister(OUTZ_H_REG)) << 8;
    newData.magneticFieldZ = mUnit * val;

    return newData;
}

}  // namespace Boardcore