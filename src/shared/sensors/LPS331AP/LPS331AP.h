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

#pragma once

#include <diagnostic/PrintLogger.h>
#include <drivers/i2c/I2C.h>
#include <sensors/Sensor.h>

#include "LPS331APData.h"

namespace Boardcore
{

class LPS331AP : public Sensor<LPS331APData>
{
public:
    enum class ODR : uint8_t
    {
        ODR_1HZ    = 0x01,
        ODR_7Hz    = 0x05,
        ODR_12_5Hz = 0x06,
        ODR_25Hz   = 0x07,
    };

    explicit LPS331AP(I2C& bus, ODR odr = ODR::ODR_25Hz);

    bool init() override;

    bool selfTest() override;

private:
    LPS331APData sampleImpl() override;

    bool checkWhoAmI();

    static constexpr uint8_t WHO_AM_I_VAL = 0x58;  ///< Who am I value

    enum Registers : uint8_t
    {
        REG_WHO_AM_I   = 0x8f,
        REG_RES_CONF   = 0x10,
        REG_CTRL_REG1  = 0x20,
        REG_PRESS_XLSB = 0x28,
    };

    I2C& bus;
    I2CDriver::I2CSlaveConfig slaveConfig{0x5c, I2CDriver::Addressing::BIT7,
                                          I2CDriver::Speed::STANDARD};

    ODR odr;

    PrintLogger logger = Logging::getLogger("lps331ap");
};

}  // namespace Boardcore
