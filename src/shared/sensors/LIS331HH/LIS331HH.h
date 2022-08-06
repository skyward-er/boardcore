/* Copyright (c) 2022 Skyward Experimental Rocketry
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

#include <drivers/spi/SPIDriver.h>
#include <sensors/Sensor.h>

#include "LIS331HHData.h"

namespace Boardcore
{

class LIS331HH : public Sensor<LIS331HHData>
{
public:
    enum OutputDataRate : uint8_t
    {
        ODR_50   = 0x00,
        ODR_100  = 0x08,
        ODR_400  = 0x10,
        ODR_1000 = 0x18,
    };

    enum FullScaleRange : uint8_t
    {
        FS_6  = 0x00,
        FS_12 = 0x10,
        FS_24 = 0x30,
    };

    LIS331HH(SPIBusInterface& bus, miosix::GpioPin cs, SPIBusConfig spiConfig);

    bool init() override;

    bool selfTest() override;

    void setOutputDataRate(OutputDataRate odr);

    void setFullScaleRange(FullScaleRange fs);

private:
    LIS331HHData sampleImpl() override;

    SPISlave slave;
    float sensitivity = 6.0 / 32767.0;

    enum Register : uint8_t
    {
        CTRL_REG1       = 0x20,
        CTRL_REG2       = 0x21,
        CTRL_REG3       = 0x22,
        CTRL_REG4       = 0x23,
        CTRL_REG5       = 0x24,
        HP_FILTER_RESET = 0x25,
        REFERENCE       = 0x26,
        STATUS_REG      = 0x27,
        OUT_X_L         = 0x28,
        OUT_X_H         = 0x29,
        OUT_Y_L         = 0x2a,
        OUT_Y_H         = 0x2b,
        OUT_Z_L         = 0x2c,
        OUT_Z_H         = 0x2d,
        INT1_CFG        = 0x30,
        INT1_SOURCE     = 0x31,
        INT1_THS        = 0x32,
        INT1_DURATION   = 0x33,
        INT2_CFG        = 0x34,
        INT2_SOURCE     = 0x35,
        INT2_THS        = 0x36,
        INT2_DURATION   = 0x37,
    };

    enum PowerMode : uint8_t
    {
        POWER_DOWN    = 0x00,
        NORMAL_MODE   = 0x20,
        LOW_POWER_0_5 = 0x40,
        LOW_POWER_1   = 0x60,
        LOW_POWER_2   = 0x80,
        LOW_POWER_5   = 0xA0,
        LOW_POWER_10  = 0xC0,
    };

    enum RegisterMask : uint8_t
    {
        CTRL_REG1_PM   = 0xE0,
        CTRL_REG1_DR   = 0x18,
        CTRL_REG1_Z_EN = 0x04,
        CTRL_REG1_Y_EN = 0x02,
        CTRL_REG1_X_EN = 0x01,

        CTRL_REG4_FS      = 0x30,
        CTRL_REG4_ST_SIGN = 0x08,
        CTRL_REG4_ST      = 0x02,
    };
};

}  // namespace Boardcore