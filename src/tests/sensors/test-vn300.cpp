/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Lorenzo Cucchi
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
#include <sensors/VN300/VN300.h>
#include <utils/Stats/Stats.h>

#include "diagnostic/CpuMeter/CpuMeter.h"
#define ENABLE_CPU_METER

using namespace miosix;
using namespace Boardcore;

int main()
{
    VN300Data sample;
    string sampleRaw;

    GpioPin u6tx1(GPIOG_BASE, 14);
    GpioPin u6rx1(GPIOG_BASE, 9);

    u6rx1.alternateFunction(8);
    u6rx1.mode(Mode::ALTERNATE);
    u6tx1.alternateFunction(8);
    u6tx1.mode(Mode::ALTERNATE);

    USART usart(USART6, 115200);
    VN300 sensor(usart, 230400, VN300::CRCOptions::CRC_ENABLE_8);

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
        printf("acc: %" PRIu64 ", %.6f, %.6f, %.6f\n",
               sample.accelerationTimestamp, sample.accelerationX,
               sample.accelerationY, sample.accelerationZ);
        printf("ang: %.6f, %.6f, %.6f\n", sample.angularSpeedX,
               sample.angularSpeedY, sample.angularSpeedZ);
        printf("ins: %.6f, %.6f, %.6f\n", sample.yaw, sample.pitch,
               sample.roll);
    }

    CpuMeter::resetCpuStats();

    Thread::sleep(1000);

    uint64_t time_start = getTick();
    for (int i = 0; i < 30; i++)
    {

        ledOn();
        sensor.sample();
        sample = sensor.getLastSample();

        printf("Sample %i\n", i);
        printf("acc: %" PRIu64 ", %.3f, %.3f, %.3f\n",
               sample.accelerationTimestamp, sample.accelerationX,
               sample.accelerationY, sample.accelerationZ);
        printf("ang: %.6f, %.6f, %.6f\n", sample.angularSpeedX,
               sample.angularSpeedY, sample.angularSpeedZ);
        printf("ins: %.6f, %.6f, %.6f\n", sample.yaw, sample.pitch,
               sample.roll);
    }
    ledOff();

    printf("Run done in %" PRIu64 " milliseconds\n", (getTick() - time_start));

    CpuMeterData cpuData = Boardcore::CpuMeter::getCpuStats();
    printf("CPU: %f\n", cpuData.mean);

    sensor.closeAndReset();
    printf("Sensor communication closed!\n");

    return 0;
}
