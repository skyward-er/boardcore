/* Copyright (c) 2023 Skyward Experimental Rocketry
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

#include "LPS22DF.h"

#include <drivers/timer/TimestampTimer.h>
#include <miosix.h>
#include <sensors/calibration/SensorDataExtra/SensorDataExtra.h>
#include <utils/Debug.h>

#include <iostream>

using namespace Eigen;

namespace Boardcore
{

static constexpr uint8_t WHO_AM_I_VALUE = 0xb4;
static constexpr uint8_t ENABLE_ONESHOT = (0b01 << 0);

LPS22DF::LPS22DF(SPIBusInterface& bus, miosix::GpioPin pin,
                 SPIBusConfig spiConfig, Config config)
    : mSlave(bus, pin, spiConfig), mConfig(config), isInitialized(false)
{
}

SPIBusConfig LPS22DF::getDefaultSPIConfig()
{
    SPIBusConfig spiConfig;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_256;
    spiConfig.mode         = SPI::Mode::MODE_3;
    spiConfig.byteOrder    = SPI::Order::LSB_FIRST;
    return spiConfig;
}

bool LPS22DF::init()
{
    if (isInitialized)
    {
        LOG_ERR(logger, "Attempted to initialized sensor twice but failed");
        lastError = ALREADY_INIT;
        return false;
    }

    {
        SPITransaction spi(mSlave);
        spi.writeRegister(IF_CTRL, if_ctrl::I2C_I3C_DIS);
    }

    // Setting the actual sensor configurations (Mode, ODR, AVG)
    if (!setConfig(mConfig))
    {
        LOG_ERR(logger, "Configuration not applied correctly");
        lastError = SensorErrors::INIT_FAIL;
        return false;
    }

    lastError     = SensorErrors::NO_ERRORS;
    isInitialized = true;
    return setConfig(mConfig);
}

bool LPS22DF::selfTest()
{
    if (!isInitialized)
    {
        LOG_ERR(logger, "Invoked selfTest() but sensor was uninitialized");
        lastError = NOT_INIT;
        return false;
    }

    // selfTest procedure does not exist for this sensor. WhoamiValue is checked
    // to assure communication.
    {
        SPITransaction spi(mSlave);
        uint8_t value = spi.readRegister(WHO_AM_I);

        if (value != WHO_AM_I_VALUE)
        {
            LOG_ERR(logger,
                    "WHO_AM_I value differs from expectation: read 0x{:x} "
                    "but expected 0x{:x}",
                    value, WHO_AM_I_VALUE);
            lastError = INVALID_WHOAMI;
            return false;
        }
    }

    return true;
}

bool LPS22DF::setConfig(const Config& config)
{
    SPITransaction spi(mSlave);

    switch (config.mode)
    {
        case Mode::ONE_SHOT_MODE:
            mConfig = {config.avg, Mode::ONE_SHOT_MODE, ODR::ONE_SHOT};
            spi.writeRegister(FIFO_CTRL, fifo_ctrl::BYPASS);
            break;

        case Mode::CONITNUOUS_MODE:
            mConfig = config;
            spi.writeRegister(FIFO_CTRL, fifo_ctrl::CONTINUOUS);
            break;

        default:
            LOG_ERR(logger, "Mode not supported");
            return false;
    }

    if (!(setAverage(mConfig.avg) && setOutputDataRate(mConfig.odr)))
    {
        LOG_ERR(logger, "Sensor not configured");
        return false;
    }

    return true;
}

bool LPS22DF::setAverage(AVG avg)
{
    SPITransaction spi(mSlave);
    spi.writeRegister(CTRL_REG1, mConfig.odr | avg);

    mConfig.avg = avg;
    return true;
}

bool LPS22DF::setOutputDataRate(ODR odr)
{
    SPITransaction spi(mSlave);
    spi.writeRegister(CTRL_REG1, odr | mConfig.avg);

    mConfig.odr = odr;
    return true;
}

float LPS22DF::convertPressure(uint8_t pressXL, uint8_t pressL, uint8_t pressH)
{
    uint32_t press_value =
        ((int32_t)((pressH << 24) | (pressL << 16) | (pressXL << 8))) >> 8;
    return ((float)press_value / pressureSensitivity);
}

float LPS22DF::convertTemperature(uint8_t tempL, uint8_t tempH)
{
    uint16_t temp_value = ((int16_t)((tempH << 8) | (tempL << 0)));
    return ((float)temp_value / temperatureSensitivity);
}

LPS22DFData LPS22DF::sampleImpl()
{
    if (!isInitialized)
    {
        LOG_ERR(logger, "Invoked sampleImpl() but sensor was uninitialized");
        lastError = NOT_INIT;
        return lastSample;
    }

    // uint8_t val[5] = {0};
    uint8_t status_val = 0;
    SPITransaction spi(mSlave);
    LPS22DFData data;

    lastError              = NO_ERRORS;
    data.pressureTimestamp = data.temperatureTimestamp =
        TimestampTimer::getTimestamp();

    if (LPS22DF::status::P_DA)
    {
        int8_t outPressL  = spi.readRegister(PRESSURE_OUT_L);
        int8_t outPressH  = spi.readRegister(PRESSURE_OUT_H);
        int8_t outPressXL = spi.readRegister(PRESSURE_OUT_XL);
        data.pressure     = convertPressure(outPressXL, outPressL, outPressH);
        if (LPS22DF::status::T_DA)
        {
            int8_t outTempL  = spi.readRegister(TEMP_OUT_L);
            int8_t outTempH  = spi.readRegister(TEMP_OUT_H);
            data.temperature = convertTemperature(outTempL, outTempH);
            data.temperature += REFERENCE_TEMPERATURE;
        }
        else
        {
            data.temperature          = lastSample.temperature;
            data.temperatureTimestamp = lastSample.temperatureTimestamp;
        }
    }
    else
    {
        lastError = NO_NEW_DATA;
        return lastSample;
    }

    return data;
}

}  // namespace Boardcore
