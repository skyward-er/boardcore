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

namespace Boardcore
{
static const uint16_t lsp28dfwAddress0{0b1011100};
static const uint16_t lsp28dfwAddress1{0b1011101};

static const uint8_t INTERRUPT_CFG{0x0B};
static const uint8_t THS_P_L{0x0C};
static const uint8_t THS_P_H{0x0D};
static const uint8_t IF_CTRL{0x0E};
static const uint8_t WHO_AM_I{0x0F};
static const uint8_t WHO_AM_I_VALUE{0xB4};
static const uint8_t CTRL_REG1_addr{0x10};  // ODR + AVG

static const uint8_t CTRL_REG2_addr{0x11};
enum CTRL_REG2 : uint8_t
{
    ONE_SHOT_START = (0b1 << 0),
    SWRESET        = (0b1 << 2),
    BDU            = (0b1 << 3),
    EN_LPFP        = (0b1 << 4),
    LFPF_CFG       = (0b1 << 5),
    FS_MODE        = (0b1 << 6),
    BOOT           = (0b1 << 7)
};

static const uint8_t CTRL_REG3_addr{0x12};
enum CTRL_REG3 : uint8_t
{
    IF_ADD_INC = (0b1 << 0),
    PP_OD      = (0b1 << 1),
    INT_H_L    = (0b1 << 3)
};

static const uint8_t CTRL_REG4_addr{0x13};
enum CTRL_REG4 : uint8_t
{
    INT_F_OVR  = (0b1 << 0),
    INT_F_WTM  = (0b1 << 1),
    INT_F_FULL = (0b1 << 2),
    INT_EN     = (0b1 << 4),
    DRDY       = (0b1 << 5),
    DRDY_PLS   = (0b1 << 6)
};

static const uint8_t FIFO_CTRL_addr{0x14};
enum FIFO_CTRL : uint8_t
{
    BYPASS               = 0b000,
    FIFO                 = 0b001,
    CONTINUOUS           = 0b010,
    BYPASS_TO_FIFO       = 0b101,
    BYPASS_TO_CONTINUOUS = 0b110,
    CONTINUOUS_TO_FIFO   = 0b111,
    STOP_ON_WTM          = (0b1 << 3)
};

static const uint8_t FIFO_WTM_addr{0x15};
static const uint8_t REF_P_L_addr{0x16};
static const uint8_t REF_P_H_addr{0x17};
static const uint8_t RPDS_L_addr{0x1a};
static const uint8_t RPDS_H_addr{0x1b};
static const uint8_t INT_SOURCE_addr{0x24};
enum INT_SOURCE : uint8_t
{
    PH      = (0b1 << 0),
    PL      = (0b1 << 1),
    IA      = (0b1 << 2),
    BOOT_ON = (0b1 << 7)
};

static const uint8_t FIFO_STATUS1_addr{0x25};
static const uint8_t FIFO_STATUS2_addr{0x26};
enum FIFO_STATUS2 : uint8_t
{
    FIFO_FULL_IA = (0b1 << 5),
    FIFO_OVR_IA  = (0b1 << 6),
    FIFO_WTM_IA  = (0b1 << 7)
};

static const uint8_t STATUS_addr{0x27};
enum STATUS : uint8_t
{
    P_DA = (0b1 << 0),
    T_DA = (0b1 << 1),
    P_OR = (0b1 << 4),
    T_OR = (0b1 << 5)
};

static const uint8_t PRESS_OUT_XL_addr{0x28};
static const uint8_t PRESS_OUT_L_addr{0x29};
static const uint8_t PRESS_OUT_H_addr{0x2a};
static const uint8_t TEMP_OUT_L_addr{0x2b};
static const uint8_t TEMP_OUT_H_addr{0x2c};
static const uint8_t FIFO_DATA_OUT_PRESS_XL_addr{0x78};
static const uint8_t FIFO_DATA_OUT_PRESS_L_addr{0x79};
static const uint8_t FIFO_DATA_OUT_PRESS_H_addr{0x7a};

LPS28DFW::LPS28DFW(I2C& i2c, bool sa0, SensorConfig sensorConfig)
    : i2c(i2c), i2cConfig{(sa0 ? lsp28dfwAddress1 : lsp28dfwAddress0),
                          I2CDriver::Addressing::BIT7,
                          I2CDriver::Speed::MAX_SPEED},
      sensorConfig(sensorConfig)
{
    pressureSensitivity = 4096;  // [LSB/hPa]
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
                sensorConfig = {newSensorConfig.fsr, newSensorConfig.avg,
                                Mode::ONE_SHOT_MODE, ODR::ONE_SHOT, false};
                fifo_ctrl |= FIFO_CTRL::BYPASS;
                break;
            case Mode::CONTINUOUS_MODE:
                sensorConfig = newSensorConfig;
                fifo_ctrl |= FIFO_CTRL::CONTINUOUS;
                break;
            default:
                LOG_ERR(logger, "Mode not supported");
                break;
        }

        if (!i2c.writeRegister(i2cConfig, FIFO_CTRL_addr, fifo_ctrl))
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
    // Trying to probe the sensor to check if it is connected
    if (!i2c.probe(i2cConfig))
    {
        LOG_ERR(logger,
                "Can't communicate with the sensor or sensor not attached");
        lastError = BUS_FAULT;
        return false;
    }

    // Checking the whoami value to assure communication
    uint8_t whoamiValue{0};
    if (!i2c.readRegister(i2cConfig, WHO_AM_I, whoamiValue))
    {
        LOG_ERR(logger, "Can't communicate with the sensor");
        lastError = BUS_FAULT;
        return false;
    }

    if (whoamiValue != WHO_AM_I_VALUE)
    {
        LOG_ERR(logger,
                "WHO_AM_I value differs from expectation: read 0x{02x} "
                "but expected 0x{02x}",
                whoamiValue, WHO_AM_I_VALUE);
        lastError = INVALID_WHOAMI;
        return false;
    }

    return true;
}

bool LPS28DFW::setAverage(AVG avg)
{
    if (!i2c.writeRegister(i2cConfig, CTRL_REG1_addr, sensorConfig.odr | avg))
    {
        lastError = BUS_FAULT;
        return false;
    }

    sensorConfig.avg = avg;
    return true;
}

bool LPS28DFW::setOutputDataRate(ODR odr)
{
    if (!i2c.writeRegister(i2cConfig, CTRL_REG1_addr, odr | sensorConfig.avg))
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
    if (!i2c.readRegister(i2cConfig, CTRL_REG2_addr, ctrl_reg2))
    {
        lastError = BUS_FAULT;
        return false;
    }

    if (fs == FullScaleRange::FS_1260)
    {
        pressureSensitivity = 4096;  // hPa
        ctrl_reg2           = (ctrl_reg2 & ~CTRL_REG2::FS_MODE);
    }
    else
    {
        pressureSensitivity = 2048;  // hPa
        ctrl_reg2           = (ctrl_reg2 | CTRL_REG2::FS_MODE);
    }

    if (!i2c.writeRegister(i2cConfig, CTRL_REG2_addr, ctrl_reg2))
    {
        lastError = BUS_FAULT;
        return false;
    }

    sensorConfig.fsr = fs;
    return true;
}

bool LPS28DFW::setDRDYInterrupt(bool drdy)
{
    if (!i2c.writeRegister(i2cConfig, CTRL_REG4_addr,
                           (drdy ? (INT_EN | DRDY) : 0)))
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
    return ((float)temp_temp) / temperatureSensitivity;
}

LPS28DFWData LPS28DFW::sampleImpl()
{
    uint8_t status = 0;
    uint8_t val[5] = {0};
    LPS28DFWData data;

    lastError              = NO_ERRORS;
    data.pressureTimestamp = data.temperatureTimestamp =
        TimestampTimer::getTimestamp();

    if (sensorConfig.odr == ODR::ONE_SHOT)
    {
        uint8_t ctrl_reg2_val{0};
        // reading 5 bytes if also Temperature new sample, otherwise only the 3
        // pressure sensors bytes
        if (!(i2c.readRegister(i2cConfig, CTRL_REG2_addr, ctrl_reg2_val) &&
              i2c.writeRegister(i2cConfig, CTRL_REG2_addr,
                                ctrl_reg2_val | CTRL_REG2::ONE_SHOT_START) &&
              i2c.readFromRegister(i2cConfig, PRESS_OUT_XL_addr, val, 5)))
        {
            lastError = BUS_FAULT;
            return lastSample;
        }

        data.pressure    = convertPressure(val[0], val[1], val[2]);
        data.temperature = convertTemperature(val[3], val[4]);

        return data;
    }

    if (!i2c.readRegister(i2cConfig, STATUS_addr, status))
    {
        lastError = BUS_FAULT;
        return lastSample;
    }

    // If pressure new data present
    if (status & STATUS::P_DA)
    {
        // reading 5 bytes if also Temperature new sample, otherwise only the 3
        // pressure sensors bytes
        if (!i2c.readFromRegister(i2cConfig, PRESS_OUT_XL_addr, val,
                                  ((status & STATUS::T_DA) ? 5 : 3)))
        {
            lastError = BUS_FAULT;
            return lastSample;
        }

        data.pressure = convertPressure(val[0], val[1], val[2]);

        // If temperature new data present
        if (status & STATUS::T_DA)
        {
            data.temperature = convertTemperature(val[3], val[4]);
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
