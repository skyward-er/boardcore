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

using namespace Boardcore::LPS22DFDefs;

namespace Boardcore
{

LPS22DF::LPS22DF(SPIBusInterface& bus, miosix::GpioPin cs)
    : slave(bus, cs, getDefaultSPIConfig())
{
}

LPS22DF::LPS22DF(SPIBusInterface& bus, miosix::GpioPin cs,
                 SPIBusConfig spiConfig, Config config)
    : slave(bus, cs, spiConfig), config(config)
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
        LOG_ERR(logger, "Attempted to initialized sensor twice");
        lastError = ALREADY_INIT;
        return false;
    }

    {
        // Disable I2C and I3C interfaces
        SPITransaction spi(slave);
        spi.writeRegister(IF_CTRL_addr, IF_CTRL::I2C_I3C_DIS);
    }

    // Setting the actual sensor configurations (Mode, ODR, AVG)
    setConfig(config);

    lastError     = SensorErrors::NO_ERRORS;
    isInitialized = true;
    return true;
}

bool LPS22DF::selfTest()
{
    // Since the sensor doesn't provide any self-test feature we just try to
    // probe the sensor and read his whoami register.
    if (!isInitialized)
    {
        LOG_ERR(logger, "Invoked selfTest() but sensor was uninitialized");
        lastError = NOT_INIT;
        return false;
    }

    {
        // Reading the whoami value to assure communication
        SPITransaction spi(slave);
        uint8_t whoamiValue = spi.readRegister(WHO_AM_I_addr);

        // Checking the whoami value to assure correct communication
        if (whoamiValue != WHO_AM_I_VALUE)
        {
            LOG_ERR(logger, "WHO_AM_I: read 0x{:x} but expected 0x{:x}",
                    whoamiValue, WHO_AM_I_VALUE);
            lastError = INVALID_WHOAMI;
            return false;
        }
    }

    return true;
}

void LPS22DF::setConfig(const Config& config)
{
    setAverage(config.avg);
    setOutputDataRate(config.odr);
}

void LPS22DF::setAverage(AVG avg)
{
    // Since the CTRL_REG1 contains only the AVG and ODR settings, we use
    // the internal driver state to set the register with the wanted ODR and
    // AVG without previously reading it. This allows to avoid a useless
    // transaction.
    {
        SPITransaction spi(slave);
        spi.writeRegister(CTRL_REG1_addr, config.odr | avg);
    }
    config.avg = avg;
}

void LPS22DF::setOutputDataRate(ODR odr)
{
    // Since the CTRL_REG1 contains only the AVG and ODR settings, we use
    // the internal driver state to set the register with the wanted ODR and
    // AVG without previously reading it. This allows to avoid a useless
    // transaction.
    {
        SPITransaction spi(slave);
        spi.writeRegister(CTRL_REG1_addr, odr | config.avg);
    }
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

    LPS22DFData data;
    SPITransaction spi(slave);
    uint8_t statusValue{0};

    if (config.odr == ODR::ONE_SHOT)
    {
        // Reading previous value of Control Register 2
        uint8_t ctrl_reg2_val = spi.readRegister(CTRL_REG2_addr);

        // Trigger sample
        spi.writeRegister(CTRL_REG2_addr,
                          ctrl_reg2_val | CTRL_REG2::ONE_SHOT_START);

        // Pool status register until the sample is ready
        do
        {
            statusValue = spi.readRegister(STATUS_addr);
        } while (!(statusValue & (STATUS::P_DA | STATUS::T_DA)));
    }
    else
    {
        // read status register value
        statusValue = spi.readRegister(STATUS_addr);
    }

    auto ts = TimestampTimer::getTimestamp();

    // Sample pressure if data is available, return last sample otherwise
    if (statusValue & STATUS::P_DA)
    {
        data.pressureTimestamp = ts;
        data.pressure = spi.readRegister24(PRESSURE_OUT_XL_addr) / PRES_SENS;
    }
    else
    {
        lastError              = NO_NEW_DATA;
        data.pressureTimestamp = lastSample.pressureTimestamp;
        data.pressure          = lastSample.pressure;
    }

    // Sample temperature if data is available, return last sample otherwise
    if (statusValue & STATUS::T_DA)
    {
        data.temperatureTimestamp = ts;
        data.temperature = spi.readRegister16(TEMP_OUT_L_addr) / TEMP_SENS;
    }
    else
    {
        data.temperatureTimestamp = lastSample.temperatureTimestamp;
        data.temperature          = lastSample.temperature;
    }

    return data;
}

}  // namespace Boardcore
