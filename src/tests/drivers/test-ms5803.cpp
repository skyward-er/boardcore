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
#include <drivers/BusTemplate.h>
#include <interfaces-impl/hwmapping.h>
#include "sensors/MS580301BA07/MS580301BA07.h"

#include <drivers/spi/SensorSpi.h>
#include <sensors/SensorSampling.h>

using namespace miosix;
using namespace miosix::interfaces;
typedef Gpio<GPIOD_BASE, 7> cs_ms58;

typedef BusSPI<1, spi1::mosi, spi1::miso, spi1::sck> busSPI1;
typedef ProtocolSPI<busSPI1, cs_ms58> spiMS58;
typedef MS580301BA07<spiMS58> ms58_t;

int main()
{
    SimpleSensorSampler sampler;

    spiMS58::init();
    ms58_t* ms58 = new ms58_t();

    Thread::sleep(100);

    if (ms58->init())
    {
        printf("MS58 Init succeeded\n");
        sampler.AddSensor(ms58);
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
        sampler.Update();

        const float* last_pressure = ms58->pressureDataPtr();
        const float* last_temp     = ms58->tempDataPtr();
        MS5803Data md              = ms58->getData();
        printf("%d,%f,%d,%f\n", (int)md.raw_press,
               *last_pressure, (int)md.raw_temp, *last_temp);

        Thread::sleep(100);
    }
}