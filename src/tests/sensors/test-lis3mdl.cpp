/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Author: Riccardo Musso
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
#include <miosix.h>
#include <sensors/LIS3MDL/LIS3MDL.h>
#include <utils/Debug.h>

using namespace Boardcore;
using namespace miosix;

int main()
{
    GpioPin cs(GPIOG_BASE, 6), miso(GPIOA_BASE, 6), mosi(GPIOA_BASE, 7),
        clk(GPIOA_BASE, 5);

    cs.mode(Mode::OUTPUT);
    cs.high();

    clk.mode(Mode::ALTERNATE);
    clk.alternateFunction(5);

    miso.mode(Mode::ALTERNATE);
    miso.alternateFunction(5);

    mosi.mode(Mode::ALTERNATE);
    mosi.alternateFunction(5);

    SPIBus bus(SPI1);

    SPIBusConfig busConfig;
    busConfig.clockDivider = SPI::ClockDivider::DIV_32;

    LIS3MDL::Config config;
    config.odr                = LIS3MDL::ODR_560_HZ;
    config.scale              = LIS3MDL::FS_16_GAUSS;
    config.temperatureDivider = 5;

    LIS3MDL sensor(bus, cs, busConfig, config);

    if (!sensor.init())
    {
        TRACE("LIS3MDL: Init failed");
        return 1;
    }

    TRACE("Doing self test!\n");
    bool ok = sensor.selfTest();
    if (!ok)
    {
        TRACE("Error: selfTest() returned false!\n");
    }

    TRACE("Now printing some sensor data:\n");
    Thread::sleep(100);

    while (true)
    {
        sensor.sample();
        LIS3MDLData data __attribute__((unused)) = sensor.getLastSample();
        TRACE("%f C | x: %f | y: %f | z %f\n", data.temperature,
              data.magneticFieldX, data.magneticFieldY, data.magneticFieldZ);
        miosix::Thread::sleep(2000);
    }
}
