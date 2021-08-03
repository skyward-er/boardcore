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
#include <drivers/spi/SensorSpi.h>
#include <interfaces-impl/hwmapping.h>
#include <sensors/SensorSampler.h>

#include "sensors/MS580301BA07/MS580301BA07.h"

using namespace miosix;
using namespace miosix::interfaces;

SPIBus bus{SPI1};
GpioPin chip_select{GPIOD_BASE, 7};

int main()
{
    // Activate the SPI bus
    {
        FastInterruptDisableLock dLock;
        RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

        // SCK, MISO, MOSI already initialized in the bsp
    }

    TimestampTimer::enableTimestampTimer();

    MS580301BA07 sensor(bus, chip_select,
                        5);  // sample temperature every 5 pressure samples

    Thread::sleep(100);

    if (sensor.init())
    {
        printf("MS58 Init succeeded\n");
    }
    else
    {
        printf("MS58 Init failed\n");

        while (!sensor.init())
        {
            printf("MS58 Init failed\n");
            Thread::sleep(1000);
        }
    }

    Thread::sleep(100);
    printf("press_timestamp,press,temp_timestamp,temp\n");

    while (true)
    {
        sensor.sample();
        Thread::sleep(100);

        const float last_pressure = sensor.getLastSample().press;
        const float last_temp     = sensor.getLastSample().temp;
        MS5803Data md             = sensor.getLastSample();
        printf("%llu,%f,%llu,%f\n", md.press_timestamp, last_pressure,
               md.temp_timestamp, last_temp);

        Thread::sleep(100);
    }
}