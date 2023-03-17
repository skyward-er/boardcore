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
#include <drivers/usart/USART.h>
#include <miosix.h>
#include <sensors/LIS2MDL/LIS2MDL.h>
#include <utils/Debug.h>

using namespace Boardcore;
using namespace miosix;

GpioPin clk(GPIOA_BASE, 5);
GpioPin miso(GPIOA_BASE, 6);
GpioPin mosi(GPIOA_BASE, 7);
GpioPin cs(GPIOA_BASE, 15);

int main()
{
    clk.mode(Mode::ALTERNATE);
    clk.alternateFunction(5);
    clk.speed(Speed::_100MHz);
    miso.mode(Mode::ALTERNATE);
    miso.alternateFunction(5);
    mosi.mode(Mode::ALTERNATE);
    mosi.alternateFunction(5);
    cs.mode(Mode::OUTPUT);
    cs.high();

    SPIBus bus(SPI1);

    SPIBusConfig busConfig = LIS2MDL::getDefaultSPIConfig();

    LIS2MDL::Config config;
    config.odr                = LIS2MDL::ODR_10_HZ;
    config.deviceMode         = LIS2MDL::MD_CONTINUOUS;
    config.temperatureDivider = 5;

    LIS2MDL sensor(bus, cs, busConfig, config);

    printf("Starting...\n");

    if (!sensor.init())
    {
        printf("LIS2MDL: Init failed\n");
        return 1;
    }
    printf("LIS2MDL: Init done\n");

    printf("Doing self test!\n");
    if (!sensor.selfTest())
    {
        printf("Error: selfTest() returned false!\n");
    }
    printf("selfTest returned true\n");
    printf("Now printing some sensor data:\n");

    while (true)
    {
        sensor.sample();
        LIS2MDLData data = sensor.getLastSample();
        printf("%f C | x: %f | y: %f | z: %f\n", data.temperature,
               data.magneticFieldX, data.magneticFieldY, data.magneticFieldZ);
        miosix::Thread::sleep(10);
    }
}
