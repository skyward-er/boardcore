/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Nuno Barcellos
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
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

    SimpleSensorSampler sampler(1, 50);

    MS580301BA07* ms58 = new MS580301BA07(bus, chip_select);

    Thread::sleep(100);

    if (ms58->init())
    {
        printf("MS58 Init succeeded\n");

        // SensorInfo : { freq, callback, is_dma, is_enabled }
        SensorInfo s_info{1, std::bind([&]() {}), false, true};
        sampler.addSensor(ms58, s_info);
    }
    else
    {
        printf("MS58 Init failed\n");

        while (!ms58->init())
        {
            printf("MS58 Init failed\n");
            Thread::sleep(1000);
        }
    }

    Thread::sleep(100);
    printf("raw_p,p,raw_t,t\n");
    while (true)
    {
        sampler.sampleAndCallback();

        const float last_pressure = ms58->getLastSample().press;
        const float last_temp     = ms58->getLastSample().temp;
        MS5803Data md             = ms58->getLastSample();
        printf("%d,%f,%d,%f\n", (int)md.raw_press, last_pressure,
               (int)md.raw_temp, last_temp);

        Thread::sleep(100);
    }
}