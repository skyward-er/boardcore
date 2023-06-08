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

LPS22DF::LPS22DF(SPIBusInterface& bus, miosix::GpioPin pin)
    : slave(bus, pin, getDefaultSPIConfig())
{
}

LPS22DF::LPS22DF(SPIBusInterface& bus, miosix::GpioPin pin,
                 SPIBusConfig spiConfig, Config config)
    : slave(bus, pin, spiConfig), config(config)
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
    SPITransaction spi(slave);

    if (isInitialized)
    {
        LOG_ERR(logger, "Attempted to initialized sensor twice");
        lastError = ALREADY_INIT;
        return false;
    }

    // Disable I2C and I3C interfaces
    spi.writeRegister(IF_CTRL_REG, I2C_I3C_DIS);

    // Setting the actual sensor configurations (Mode, ODR, AVG)
    setConfig(config);

    // TODO: Read back registers to check configuration

    isInitialized = true;
    return true;
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
        SPITransaction spi(slave);
        uint8_t value = spi.readRegister(WHO_AM_I_REG);

        if (value != WHO_AM_I_VALUE)
        {
            LOG_ERR(logger, "WHO_AM_I: read 0x{:x} but expected 0x{:x}", value,
                    WHO_AM_I_VALUE);
            lastError = INVALID_WHOAMI;
            return false;
        }
    }

    return true;
}

void LPS22DF::setConfig(const Config& config)
{
    SPITransaction spi(slave);

    setAverage(config.avg);
    setOutputDataRate(config.odr);
}

void LPS22DF::setAverage(AVG avg)
{
    SPITransaction spi(slave);

    spi.writeRegister(CTRL1_REG, avg);

    config.avg = avg;
}

void LPS22DF::setOutputDataRate(ODR odr)
{
    SPITransaction spi(slave);

    spi.writeRegister(CTRL1_REG, odr);

    config.odr = odr;
}

LPS22DFData LPS22DF::sampleImpl()
{
    if (!isInitialized)
    {
        LOG_ERR(logger, "Invoked sampleImpl() but sensor was not initialized");
        lastError = NOT_INIT;
        return lastSample;
    }

    SPITransaction spi(slave);
    LPS22DFData data;

    // TODO: Handle ONE SHOT mode
    if (config.odr == ODR::ONE_SHOT)
    {
        // Trigger sample
        spi.writeRegister(CTRL2_REG, ONE_SHOT_TRIGGER);

        // Pool status register until the sample is ready
        while ((spi.readRegister16(STATUS_REG) & 0x3) == 0)
            ;
    }

    data.pressureTimestamp    = TimestampTimer::getTimestamp();
    data.temperatureTimestamp = data.pressureTimestamp;

    data.pressure    = spi.readRegister24(PRESSURE_OUT_XL_REG) / PRES_SENS;
    data.temperature = spi.readRegister16(TEMP_OUT_L_REG) / TEMP_SENS;

    return data;
}

}  // namespace Boardcore
