
/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include "LPS331AP.h"

#include <drivers/timer/TimestampTimer.h>

namespace Boardcore
{

LPS331AP::LPS331AP(I2C& bus, ODR odr) : bus(bus), odr(odr) {}

bool LPS331AP::init()
{
    if (!checkWhoAmI())
    {
        return false;
    }

    uint8_t ctrlReg1 = 0;
    ctrlReg1 |= 0x80;  // Active mode
    ctrlReg1 |= static_cast<uint8_t>(odr) << 4;

    if (!bus.writeRegister(slaveConfig, REG_CTRL_REG1, ctrlReg1))
    {

        lastError = SensorErrors::BUS_FAULT;
        return false;
    }

    // Maximum possible oversampling to lower the noise
    uint8_t ctrlConf = odr != ODR::ODR_25Hz ? 0x69 : 0x7a;

    if (!bus.writeRegister(slaveConfig, REG_RES_CONF, ctrlConf))
    {
        lastError = SensorErrors::BUS_FAULT;
        return false;
    }

    return true;
}

bool LPS331AP::selfTest() { return checkWhoAmI(); }

LPS331APData LPS331AP::sampleImpl()
{
    uint8_t buffer[5];
    if (bus.readFromRegister(slaveConfig, REG_PRESS_XLSB, buffer, 5))
    {
        LPS331APData data;

        int32_t pressure = 0;
        pressure |= static_cast<uint32_t>(buffer[0]) << 16;
        pressure |= static_cast<uint32_t>(buffer[1]) << 8;
        pressure |= static_cast<uint32_t>(buffer[2]);

        int32_t temperature = 0;
        temperature |= static_cast<uint32_t>(buffer[3]) << 8;
        temperature |= static_cast<uint32_t>(buffer[4]);

        data.pressureTimestamp    = TimestampTimer::getTimestamp();
        data.temperatureTimestamp = TimestampTimer::getTimestamp();
        data.pressure             = pressure / 4096.0f;
        data.temperature          = temperature / 480.0f + 42.5f;

        return data;
    }
    else
    {
        lastError = SensorErrors::BUS_FAULT;
        return lastSample;
    }
}

bool LPS331AP::checkWhoAmI()
{
    uint8_t whoAmIValue;

    if (bus.readRegister(slaveConfig, REG_WHO_AM_I, whoAmIValue))
    {
        if (whoAmIValue == WHO_AM_I_VAL)
        {
            return true;
        }
        else
        {
            LOG_ERR(logger, "Invalid WHO AM I");
            lastError = SensorErrors::INVALID_WHOAMI;
            return false;
        }
    }
    else
    {
        lastError = SensorErrors::BUS_FAULT;
        return false;
    }
}

}  // namespace Boardcore
