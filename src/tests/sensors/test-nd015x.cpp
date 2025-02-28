/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Pietro Bortolus
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

#include <iostream>

#include "sensors/ND015X/ND015A.h"

using namespace miosix;
using namespace Boardcore;

int main()
{
    miosix::GpioPin cs(GPIOE_BASE, 3);

    SPIBusConfig spiConfig;
    SPIBus bus(SPI1);

    ND015A sensor(bus, cs, spiConfig);
    ND015XData sensorData;

    sensor.setOutputDataRate(100);
    sensor.setBWLimitFilter(ND015A::BWLimitFilter::BWL_100);
    sensor.setIOWatchdog(ND015A::IOWatchdogEnable::ENABLED);
    sensor.setNotch(ND015A::NotchEnable::ENABLED);

    while (true)
    {
        sensor.sample();
        sensorData = sensor.getLastSample();
        std::cout << "New data: " << sensorData.header() << std::endl;

        sleep(100);
    }

    return 0;
}

