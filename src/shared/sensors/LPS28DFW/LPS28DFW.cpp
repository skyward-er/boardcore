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

namespace Boardcore
{
LPS28DFW::LPS28DFW(I2C& i2c, SensorConfig sensorConfig)
    : i2cConfig{sensorConfig.sa0 ? LPS28DFWDefs::lsp28dfwAddress1
                                 : LPS28DFWDefs::lsp28dfwAddress0,
                I2CDriver::Addressing::BIT7, I2CDriver::Speed::MAX_SPEED},
      i2c(i2c), sensorConfig(sensorConfig)
{
    pressureSensitivity = (sensorConfig.fsr == FullScaleRange::FS_1260
                               ? LPS28DFWDefs::pressureSensitivity1260hPa
                               : LPS28DFWDefs::pressureSensitivity4060hPa);
}

bool LPS28DFW::init()
{
    if (isInitialized)
    {
        LOG_ERR(logger, "Attempted to initialized sensor twice but failed");
        lastError = ALREADY_INIT;
        return false;
    }

    // Checking the whoami value to assure communication
    if (!selfTest())
    {
        LOG_ERR(logger, "Self-test failed");
        lastError = SELF_TEST_FAIL;
        return false;
    }

    // Setting the actual sensor configurations (Mode, ODR, AVG, FSR, DRDY)
    if (!setConfig(sensorConfig))
    {
        LOG_ERR(logger, "Configuration not applied correctly");
        lastError = SensorErrors::INIT_FAIL;
        return false;
    }

    lastError     = SensorErrors::NO_ERRORS;
    isInitialized = true;
    return true;
}

bool LPS28DFW::setConfig(const SensorConfig& newSensorConfig)
{
    {
        // Setting the mode and checking for consistency in the settings. If
        // ONE_SHOT mode, other settings adapted to match this.
        uint8_t fifo_ctrl{0};
        switch (newSensorConfig.mode)
        {
            case Mode::ONE_SHOT_MODE:
                sensorConfig = {newSensorConfig.sa0, newSensorConfig.fsr,
                                newSensorConfig.avg, Mode::ONE_SHOT_MODE,
                                ODR::ONE_SHOT,       false};
                fifo_ctrl |= LPS28DFWDefs::FIFO_CTRL::BYPASS;
                break;
            case Mode::CONTINUOUS_MODE:
                sensorConfig = newSensorConfig;
                fifo_ctrl |= LPS28DFWDefs::FIFO_CTRL::CONTINUOUS;
                break;
            default:
                LOG_ERR(logger, "Mode not supported");
                break;
        }

        if (!i2c.writeRegister(i2cConfig, LPS28DFWDefs::FIFO_CTRL_addr,
                               fifo_ctrl))
        {
            lastError = BUS_FAULT;
            return false;
        }
    }

    if (!(setFullScaleRange(sensorConfig.fsr) && setAverage(sensorConfig.avg) &&
          setOutputDataRate(sensorConfig.odr) &&
          setDRDYInterrupt(sensorConfig.drdy)))
    {
        LOG_ERR(logger, "Sensor not configured");
        return false;
    }

    return true;
}

bool LPS28DFW::selfTest()
{
    // Since the sensor doesn't provide any self-test feature we just try to
    // probe the sensor and read his whoami register.

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
    if (!i2c.readRegister(i2cConfig, LPS28DFWDefs::WHO_AM_I, whoamiValue))
    {
        LOG_ERR(logger, "Can't communicate with the sensor");
        lastError = BUS_FAULT;
        return false;
    }

    // Checking the whoami value to assure correct communication
    if (whoamiValue != LPS28DFWDefs::WHO_AM_I_VALUE)
    {
        LOG_ERR(logger,
                "WHO_AM_I value differs from expectation: read 0x{02x} "
                "but expected 0x{02x}",
                whoamiValue, LPS28DFWDefs::WHO_AM_I_VALUE);
        lastError = INVALID_WHOAMI;
        return false;
    }

    return true;
}

bool LPS28DFW::setAverage(AVG avg)
{
    // Since the CTRL_REG1 contains only the AVG and ODR settings, we use the
    // internal driver state to set the register with the wanted ODR and AVG
    // without previously reading it. This allows to avoid a useless
    // transaction.
    if (!i2c.writeRegister(i2cConfig, LPS28DFWDefs::CTRL_REG1_addr,
                           sensorConfig.odr | avg))
    {
        lastError = BUS_FAULT;
        return false;
    }

    sensorConfig.avg = avg;
    return true;
}

bool LPS28DFW::setOutputDataRate(ODR odr)
{
    // Since the CTRL_REG1 contains only the AVG and ODR settings, we use the
    // internal driver state to set the register with the wanted ODR and AVG
    // without previously reading it. This allows to avoid a useless
    // transaction.
    if (!i2c.writeRegister(i2cConfig, LPS28DFWDefs::CTRL_REG1_addr,
                           odr | sensorConfig.avg))
    {
        lastError = BUS_FAULT;
        return false;
    }

    sensorConfig.odr = odr;
    return true;
}

bool LPS28DFW::setFullScaleRange(FullScaleRange fs)
{
    uint8_t ctrl_reg2;
    if (!i2c.readRegister(i2cConfig, LPS28DFWDefs::CTRL_REG2_addr, ctrl_reg2))
    {
        lastError = BUS_FAULT;
        return false;
    }

    if (fs == FullScaleRange::FS_1260)
    {
        pressureSensitivity = LPS28DFWDefs::pressureSensitivity1260hPa;
        ctrl_reg2           = (ctrl_reg2 & ~LPS28DFWDefs::CTRL_REG2::FS_MODE);
    }
    else
    {
        pressureSensitivity = LPS28DFWDefs::pressureSensitivity4060hPa;
        ctrl_reg2           = (ctrl_reg2 | LPS28DFWDefs::CTRL_REG2::FS_MODE);
    }

    if (!i2c.writeRegister(i2cConfig, LPS28DFWDefs::CTRL_REG2_addr, ctrl_reg2))
    {
        lastError = BUS_FAULT;
        return false;
    }

    sensorConfig.fsr = fs;
    return true;
}

bool LPS28DFW::setDRDYInterrupt(bool drdy)
{
    if (!i2c.writeRegister(
            i2cConfig, LPS28DFWDefs::CTRL_REG4_addr,
            (drdy ? (LPS28DFWDefs::INT_EN | LPS28DFWDefs::DRDY) : 0)))
    {
        lastError = BUS_FAULT;
        return false;
    }

    sensorConfig.drdy = drdy;
    return true;
}

float LPS28DFW::convertPressure(uint8_t pressXL, uint8_t pressL, uint8_t pressH)
{
    // Pressure conversion
    // sign extending the 27-bit value: shifting to the right a signed type
    // extends its sign. So positioning the bytes shifted to the left of 8
    // bits, casting the result in a signed int8_t and then shifting the
    // result to the right of 8 bits will make the work.
    int32_t press_temp =
        ((int32_t)((pressH << 24) | (pressL << 16) | (pressXL << 8))) >> 8;
    return ((float)press_temp / pressureSensitivity);
}

float LPS28DFW::convertTemperature(uint8_t tempL, uint8_t tempH)
{
    // Temperature conversion
    int16_t temp_temp = (tempH << 8) | (tempL << 0);
    return ((float)temp_temp) / LPS28DFWDefs::temperatureSensitivity;
}

LPS28DFWData LPS28DFW::sampleImpl()
{
    uint8_t status = 0;
    uint8_t val[5] = {0};
    LPS28DFWData data;

    lastError              = NO_ERRORS;
    data.pressureTimestamp = data.temperatureTimestamp =
        TimestampTimer::getTimestamp();

    if (sensorConfig.mode == Mode::ONE_SHOT_MODE)
    {
        uint8_t ctrl_reg2_val{0};

        // Reading always 5 bytes because in one-shot mode the sensor samples
        // both pressure and temperature
        if (!(i2c.readRegister(i2cConfig, LPS28DFWDefs::CTRL_REG2_addr,
                               ctrl_reg2_val) &&
              i2c.writeRegister(
                  i2cConfig, LPS28DFWDefs::CTRL_REG2_addr,
                  ctrl_reg2_val | LPS28DFWDefs::CTRL_REG2::ONE_SHOT_START) &&
              i2c.readFromRegister(i2cConfig, LPS28DFWDefs::PRESS_OUT_XL_addr,
                                   val, 5)))
        {
            lastError = BUS_FAULT;
            return lastSample;
        }

        data.pressure    = convertPressure(val[0], val[1], val[2]);
        data.temperature = convertTemperature(val[3], val[4]);

        return data;
    }

    if (!i2c.readRegister(i2cConfig, LPS28DFWDefs::STATUS_addr, status))
    {
        lastError = BUS_FAULT;
        return lastSample;
    }

    // If pressure new data present
    if (!(status & LPS28DFWDefs::STATUS::P_DA))
    {
        lastError = NO_NEW_DATA;
        return lastSample;
    }

    // reading 5 bytes if also Temperature new sample, otherwise only the 3
    // pressure bytes
    if (!i2c.readFromRegister(i2cConfig, LPS28DFWDefs::PRESS_OUT_XL_addr, val,
                              ((status & LPS28DFWDefs::STATUS::T_DA) ? 5 : 3)))
    {
        lastError = BUS_FAULT;
        return lastSample;
    }

    data.pressure = convertPressure(val[0], val[1], val[2]);

    // If temperature new data present
    if (status & LPS28DFWDefs::STATUS::T_DA)
    {
        data.temperature = convertTemperature(val[3], val[4]);
    }
    else
    {
        data.temperature          = lastSample.temperature;
        data.temperatureTimestamp = lastSample.temperatureTimestamp;
    }

    return data;
}

}  // namespace Boardcore
