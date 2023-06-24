/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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

#include "LPS28DFW.h"

#include <drivers/timer/TimestampTimer.h>

#include "LPS28DFWDefs.h"

using namespace Boardcore::LPS28DFWDefs;

namespace Boardcore
{

LPS28DFW::LPS28DFW(I2C& i2c, Config config)
    : i2c(i2c), i2cConfig{config.sa0 ? I2C_ADDR_1 : I2C_ADDR_0,
                          I2CDriver::Addressing::BIT7,
                          I2CDriver::Speed::MAX_SPEED},
      config(config)
{
    pressureSensitivity = config.fsr == FullScaleRange::FS_1260
                              ? PRES_SENS_1260hPa
                              : PRES_SENS_4060hPa;
}

bool LPS28DFW::init()
{
    if (isInitialized)
    {
        LOG_ERR(logger, "Attempted to initialized sensor twice but failed");
        lastError = ALREADY_INIT;
        return false;
    }

    // Setting the actual sensor configurations (Mode, ODR, AVG, FSR, DRDY)
    if (!setConfig(config))
    {
        LOG_ERR(logger, "Configuration not applied correctly");
        lastError = SensorErrors::INIT_FAIL;
        return false;
    }

    lastError     = SensorErrors::NO_ERRORS;
    isInitialized = true;
    return true;
}

bool LPS28DFW::selfTest()
{
    // Since the sensor doesn't provide any self-test feature we just try to
    // probe the sensor and read his whoami register.
    if (!isInitialized)
    {
        LOG_ERR(logger, "Invoked selfTest() but sensor was uninitialized");
        lastError = NOT_INIT;
        return false;
    }

    // Trying to probe the sensor to check if it is connected
    if (!i2c.probe(i2cConfig))
    {
        LOG_ERR(logger,
                "Can't communicate with the sensor or sensor not attached");
        lastError = BUS_FAULT;
        return false;
    }

    // Reading the whoami value to assure communication
    uint8_t whoamiValue{0};
    if (!i2c.readRegister(i2cConfig, WHO_AM_I, whoamiValue))
    {
        LOG_ERR(logger, "Can't communicate with the sensor");
        lastError = BUS_FAULT;
        return false;
    }

    // Checking the whoami value to assure correct communication
    if (whoamiValue != WHO_AM_I_VALUE)
    {
        LOG_ERR(logger, "WHO_AM_I: read 0x{:x} but expected 0x{:x}",
                whoamiValue, WHO_AM_I_VALUE);
        lastError = INVALID_WHOAMI;
        return false;
    }

    return true;
}

bool LPS28DFW::setConfig(const Config& config)
{
    /**
     * Setting the FIFO_CTRL register to the correct mode (according to the ODR
     * set: BYPASS for the one shot mode and CONTINUOUS for the continuous
     * mode).
     */
    if (!i2c.writeRegister(
            i2cConfig, FIFO_CTRL,
            (config.odr == ODR::ONE_SHOT ? FIFO_CTRL::BYPASS
                                         : FIFO_CTRL::CONTINUOUS)))
    {
        lastError = BUS_FAULT;
        return false;
    }

    if (!setFullScaleRange(config.fsr) || !setAverage(config.avg) ||
        !setOutputDataRate(config.odr))
    {
        LOG_ERR(logger, "Sensor not configured");
        return false;
    }

    return true;
}

bool LPS28DFW::setAverage(AVG avg)
{
    /**
     * Since the CTRL_REG1 contains only the AVG and ODR settings, we use the
     * internal driver state to set the register with the wanted ODR and AVG
     * without previously reading it. This allows to avoid a useless
     * transaction.
     */
    if (!i2c.writeRegister(i2cConfig, CTRL_REG1, config.odr | avg))
    {
        lastError = BUS_FAULT;
        return false;
    }

    config.avg = avg;
    return true;
}

bool LPS28DFW::setOutputDataRate(ODR odr)
{
    /**
     * Since the CTRL_REG1 contains only the AVG and ODR settings, we use the
     * internal driver state to set the register with the wanted ODR and AVG
     * without previously reading it. This allows to avoid a useless
     * transaction.
     */
    if (!i2c.writeRegister(i2cConfig, CTRL_REG1, odr | config.avg))
    {
        lastError = BUS_FAULT;
        return false;
    }

    config.odr = odr;
    return true;
}

bool LPS28DFW::setFullScaleRange(FullScaleRange fs)
{
    uint8_t ctrl_reg2;
    if (!i2c.readRegister(i2cConfig, CTRL_REG2, ctrl_reg2))
    {
        lastError = BUS_FAULT;
        return false;
    }

    if (fs == FullScaleRange::FS_1260)
    {
        pressureSensitivity = PRES_SENS_1260hPa;
        ctrl_reg2           = (ctrl_reg2 & ~CTRL_REG2::FS_MODE);
    }
    else
    {
        pressureSensitivity = PRES_SENS_4060hPa;
        ctrl_reg2           = (ctrl_reg2 | CTRL_REG2::FS_MODE);
    }

    if (!i2c.writeRegister(i2cConfig, CTRL_REG2, ctrl_reg2))
    {
        lastError = BUS_FAULT;
        return false;
    }

    config.fsr = fs;
    return true;
}

LPS28DFWData LPS28DFW::sampleImpl()
{
    if (!isInitialized)
    {
        LOG_ERR(logger, "Invoked sampleImpl() but sensor was not initialized");
        lastError = NOT_INIT;
        return lastSample;
    }

    LPS28DFWData data;
    uint8_t statusValue = 0;

    if (config.odr == ODR::ONE_SHOT)
    {
        // Reading previous value of Control Register 2
        uint8_t ctrl_reg2_val;
        if (!i2c.readRegister(i2cConfig, CTRL_REG2, ctrl_reg2_val))
        {
            lastError = BUS_FAULT;
            return lastSample;
        }

        // Trigger sample
        if (!i2c.writeRegister(i2cConfig, CTRL_REG2,
                               ctrl_reg2_val | CTRL_REG2::ONE_SHOT_START))
        {
            lastError = BUS_FAULT;
            return lastSample;
        }

        // Pull status register until the sample is ready
        do
        {
            if (!i2c.readRegister(i2cConfig, STATUS, statusValue))
            {
                lastError = BUS_FAULT;
                return lastSample;
            }
        } while (!(statusValue & (STATUS::P_DA | STATUS::T_DA)));
    }
    else
    {
        // Read status register value
        if (!i2c.readRegister(i2cConfig, STATUS, statusValue))
        {
            lastError = BUS_FAULT;
            return lastSample;
        }
    }

    auto ts = TimestampTimer::getTimestamp();

    // Sample pressure if data is available, return last sample otherwise
    if (statusValue & STATUS::P_DA)
    {
        uint32_t rawPressure;
        if (!i2c.readRegister24(i2cConfig, PRESS_OUT_XL, rawPressure))
        {
            lastError = BUS_FAULT;
            return lastSample;
        }

        data.pressureTimestamp = ts;
        data.pressure          = rawPressure / pressureSensitivity;
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
        uint16_t rawTemperature;
        if (!i2c.readRegister16(i2cConfig, TEMP_OUT_L, rawTemperature))
        {
            lastError = BUS_FAULT;
            return lastSample;
        }

        data.temperatureTimestamp = ts;
        data.temperature          = rawTemperature / TEMP_SENS;
    }
    else
    {
        data.temperatureTimestamp = lastSample.temperatureTimestamp;
        data.temperature          = lastSample.temperature;
    }

    return data;
}

}  // namespace Boardcore
