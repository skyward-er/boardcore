/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Lorenzo Cucchi, Fabrizio Monti
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

#include <drivers/timer/TimestampTimer.h>
#include <inttypes.h>
#include <sensors/Vectornav/VN300/VN300.h>
#include <utils/Stats/Stats.h>

#include <iostream>

#include "diagnostic/CpuMeter/CpuMeter.h"
#define ENABLE_CPU_METER

using namespace miosix;
using namespace Boardcore;

int main()
{
    VN300Data sample;
    string sampleRaw;

    GpioPin u6tx1(GPIOA_BASE, 2);
    GpioPin u6rx1(GPIOA_BASE, 3);

    u6rx1.alternateFunction(7);
    u6rx1.mode(Mode::ALTERNATE);
    u6tx1.alternateFunction(7);
    u6tx1.mode(Mode::ALTERNATE);

    const int baud = 921600;
    USART usart(USART2, baud);
    VN300 sensor(usart, baud, VN300Defs::SamplingMethod::BINARY,
                 VN300::CRCOptions::CRC_ENABLE_8);

    // Let the sensor start up
    Thread::sleep(1000);

    printf("Initializing sensor\n");
    if (!sensor.init())
    {
        printf("Error initializing the sensor!\n");
        return 0;
    }

    printf("Running self-test\n");
    if (!sensor.selfTest())
    {
        printf("Unable to execute self-test\n");
        return 0;
    }

    // Sample and print 10 samples
    for (int i = 0; i < 10; i++)
    {
        sensor.sample();
        sample = sensor.getLastSample();

        printf("acc: %f, %f, %f\n", sample.accelerationX, sample.accelerationY,
               sample.accelerationZ);
        printf("gyr: %f, %f, %f\n", sample.angularSpeedX, sample.angularSpeedY,
               sample.angularSpeedZ);
        printf("magn: %f, %f, %f\n", sample.magneticFieldX,
               sample.magneticFieldY, sample.magneticFieldZ);
        printf("attitude: %f, %f, %f\n", sample.yaw, sample.pitch, sample.roll);

        Thread::sleep(500);
    }

    sensor.closeAndReset();
    printf("Sensor communication closed!\n");

    return 0;
}
