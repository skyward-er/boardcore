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

#include <drivers/spi/SPIDriver.h>
#include <drivers/timer/TimestampTimer.h>
#include <miosix.h>
#include <sensors/LIS331HH/LIS331HH.h>

using namespace miosix;
using namespace Boardcore;

int main()
{
    // GpioPin cs(GPIOE_BASE, 4);
    // GpioPin sck(GPIOE_BASE, 2);
    // GpioPin miso(GPIOE_BASE, 5);
    // GpioPin mosi(GPIOE_BASE, 6);

    // sck.mode(miosix::Mode::ALTERNATE);
    // sck.alternateFunction(5);
    // miso.mode(miosix::Mode::ALTERNATE);
    // miso.alternateFunction(5);
    // mosi.mode(miosix::Mode::ALTERNATE);
    // mosi.alternateFunction(5);
    // cs.mode(miosix::Mode::OUTPUT);
    // cs.high();

    // SPIBus bus(SPI4);

    GpioPin cs = sensors::lis331hh::cs::getPin();
    SPIBus bus(SPI2);

    SPIBusConfig config;
    LIS331HH lis(bus, cs, config);

    lis.init();
    lis.setOutputDataRate(LIS331HH::ODR_1000);
    lis.setFullScaleRange(LIS331HH::FS_24);

    while (true)
    {
        lis.sample();
        auto sample = lis.getLastSample();

        printf("[%.2f] x: % 5.2f, y: % 5.2f, z: % 5.2f\n",
               sample.accelerationTimestamp / 1e6, sample.accelerationX,
               sample.accelerationY, sample.accelerationZ);

        Thread::sleep(100);
    }
}