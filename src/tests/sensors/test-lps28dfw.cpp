/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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

#include <drivers/interrupt/external_interrupts.h>

#include <iostream>

#include "miosix.h"
#include "sensors/LPS28DFW/LPS28DFW.h"
#include "sensors/LPS28DFW/LPS28DFWData.h"
#include "string"
#include "string.h"
#include "thread"

using namespace std;
using namespace miosix;
using namespace Boardcore;

GpioPin scl(GPIOB_BASE, 8);
GpioPin sda(GPIOB_BASE, 9);

int main()
{
    I2C i2c(I2C1, scl, sda);

    LPS28DFW::Config config;
    config.sa0 = false;
    config.fsr = LPS28DFW::FullScaleRange::FS_1260;
    config.odr = LPS28DFW::ODR::ONE_SHOT;
    config.avg = LPS28DFW::AVG::AVG_64;

    LPS28DFW sensor(i2c, config);

    printf("Starting...\n");

    if (!sensor.init())
    {
        printf("Init failed\n");
        return 0;
    }

    if (!sensor.selfTest())
    {
        printf("Error: selfTest() returned false!\n");
        return 0;
    }

    printf("Trying one shot mode for 10 seconds\n");
    for (uint8_t i = 0; i < 10 * 10; i++)
    {
        sensor.sample();
        LPS28DFWData data = sensor.getLastSample();

        printf("%.2f C | %.2f Pa\n", data.temperature, data.pressure);

        miosix::Thread::sleep(100);
    }

    printf("Now setting 10Hz continuous mode\n");
    sensor.setOutputDataRate(LPS28DFW::ODR::ODR_10);
    while (true)
    {
        sensor.sample();
        LPS28DFWData data = sensor.getLastSample();

        printf("%.2f C | %.2f Pa\n", data.temperature, data.pressure);

        miosix::Thread::sleep(100);
    }

    return 0;
}
