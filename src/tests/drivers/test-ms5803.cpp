/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Author: Nuno Barcellos
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

#include <Common.h>
#include <drivers/spi/SPIDriver.h>
#include <interfaces-impl/hwmapping.h>  // TODO: Remove and add gpio pins
#include <sensors/MS5803/MS5803.h>
#include <sensors/SensorSampler.h>

using namespace Boardcore;
using namespace miosix;

/**
 * This test is intended to be run on the Death Stack X
 */

int main()
{
    TimestampTimer::enableTimestampTimer();

    SPIBusConfig spiConfig;
    SPIBus spiBus(SPI1);
    SPISlave spiSlave(spiBus, miosix::sensors::ms5803::cs::getPin(), spiConfig);

    // Sample temperature every 5 pressure samples
    MS5803 sensor(spiSlave, 10);

    Thread::sleep(100);

    if (!sensor.init())
    {
        printf("MS5803 Init failed\n");
    }

    Thread::sleep(100);
    printf("press_timestamp,press,temp_timestamp,temp\n");

    while (true)
    {
        sensor.sample();

        MS5803Data data = sensor.getLastSample();
        printf("%llu,%f,%llu,%f\n", data.press_timestamp, data.press,
               data.temp_timestamp, data.temp);

        Thread::sleep(10);
    }
}