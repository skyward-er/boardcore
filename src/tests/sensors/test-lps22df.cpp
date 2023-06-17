/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Giulia Ghirardini
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
#include <sensors/LPS22DF/LPS22DF.h>
#include <utils/Debug.h>

using namespace Boardcore;
using namespace miosix;

GpioPin clk(GPIOA_BASE, 5);
GpioPin miso(GPIOA_BASE, 6);
GpioPin mosi(GPIOA_BASE, 7);
GpioPin cs(GPIOD_BASE, 14);

int main()
{
    clk.mode(Mode::ALTERNATE);
    clk.alternateFunction(5);
    miso.mode(Mode::ALTERNATE);
    miso.alternateFunction(5);
    mosi.mode(Mode::ALTERNATE);
    mosi.alternateFunction(5);
    cs.mode(Mode::OUTPUT);
    cs.high();

    SPIBus bus(SPI1);

    LPS22DF::Config config;
    config.odr = LPS22DF::ONE_SHOT;
    config.avg = LPS22DF::AVG_128;

    LPS22DF sensor(bus, cs, LPS22DF::getDefaultSPIConfig(), config);

    printf("Starting...\n");

    if (!sensor.init())
    {
        printf("Init failed\n");
        return 0;
    }

    if (!sensor.selfTest())
    {
        printf("Error: selfTest() returned false!\n");
        // return 0;
    }

    printf("Trying one shot mode for 10 seconds\n");
    for (int i = 0; i < 10 * 10; i++)
    {
        sensor.sample();
        LPS22DFData data = sensor.getLastSample();

        printf("%.2f C | %.2f Pa\n", data.temperature, data.pressure);

        miosix::Thread::sleep(100);
    }

    printf("Now setting 10Hz continuous mode\n");
    sensor.setOutputDataRate(LPS22DF::ODR_10);
    while (true)
    {
        sensor.sample();
        LPS22DFData data = sensor.getLastSample();

        printf("%.2f C | %.2f Pa\n", data.temperature, data.pressure);

        miosix::Thread::sleep(100);
    }
}
