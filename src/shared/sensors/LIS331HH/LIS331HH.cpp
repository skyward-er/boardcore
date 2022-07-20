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

    spi.writeRegister(CTRL_REG1, 0x20);
    printf("CTRL_REG1: %X\n", spi.readRegister(CTRL_REG1));

    return true;
}

bool LIS331HH::selfTest() { return true; }

LIS331HHData LIS331HH::sampleImpl()
{
    uint16_t val;
    LIS331HHData data;

    data.accelerationTimestamp = TimestampTimer::getTimestamp();

    SPITransaction spi(slave);

    val = spi.readRegister(OUT_X_L);
    val |= spi.readRegister(OUT_X_H) << 8;
    data.accelerationX = 6.0 / 65536.0 * val;
    printf("%X\t", val);

    val = spi.readRegister(OUT_Y_L);
    val |= spi.readRegister(OUT_Y_H) << 8;
    data.accelerationX = 6.0 / 65536.0 * val;
    printf("%X\t", val);

    val = spi.readRegister(OUT_Z_L);
    val |= spi.readRegister(OUT_Z_H) << 8;
    data.accelerationX = 6.0 / 65536.0 * val;
    printf("%X\t", val);

    return data;
}

}  // namespace Boardcore