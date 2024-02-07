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

#include "LIS331HH.h"

#include <drivers/timer/TimestampTimer.h>

namespace Boardcore
{

LIS331HH::LIS331HH(SPIBusInterface& bus, miosix::GpioPin cs,
                   SPIBusConfig spiConfig)
    : slave(bus, cs, spiConfig)
{
}

bool LIS331HH::init()
{
    SPITransaction spi(slave);

    spi.writeRegister(CTRL_REG1, NORMAL_MODE | CTRL_REG1_Z_EN | CTRL_REG1_Y_EN |
                                     CTRL_REG1_X_EN);

    return true;
}

bool LIS331HH::selfTest() { return true; }

void LIS331HH::setOutputDataRate(OutputDataRate odr)
{
    SPITransaction spi(slave);

    uint8_t ctrl1 = spi.readRegister(CTRL_REG1);
    ctrl1         = (ctrl1 & ~CTRL_REG1_DR) | odr;
    spi.writeRegister(CTRL_REG1, ctrl1);
}

void LIS331HH::setFullScaleRange(FullScaleRange fs)
{
    SPITransaction spi(slave);

    uint8_t ctrl4 = spi.readRegister(CTRL_REG4);
    ctrl4         = (ctrl4 & ~CTRL_REG4_FS) | fs;
    spi.writeRegister(CTRL_REG4, ctrl4);

    if (fs == FS_6)
        sensitivity = 6.0 / 32767.0;
    else if (fs == FS_12)
        sensitivity = 12.0 / 32767.0;
    else
        sensitivity = 24.0 / 32767.0;
}

LIS331HHData LIS331HH::sampleImpl()
{
    int16_t val;
    LIS331HHData data;

    data.accelerationTimestamp = TimestampTimer::getTimestamp();

    SPITransaction spi(slave);

    spi.readRegisters(0x40 | OUT_X_L, reinterpret_cast<uint8_t*>(&val), 2);
    data.accelerationX = MeterPerSecondSquared(sensitivity * val);

    spi.readRegisters(0x40 | OUT_Y_L, reinterpret_cast<uint8_t*>(&val), 2);
    data.accelerationY = MeterPerSecondSquared(sensitivity * val);

    spi.readRegisters(0x40 | OUT_Z_L, reinterpret_cast<uint8_t*>(&val), 2);
    data.accelerationZ = MeterPerSecondSquared(sensitivity * val);

    return data;
}

}  // namespace Boardcore
