/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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
#include <sensors/Vectornav/VN100/VN100.h>

using namespace miosix;
using namespace Boardcore;

int main()
{
    VN100Data sample;
    string sampleRaw;

    GpioPin u2tx1(GPIOA_BASE, 2);
    GpioPin u2rx1(GPIOA_BASE, 3);

    u2rx1.alternateFunction(7);
    u2rx1.mode(Mode::ALTERNATE);
    u2tx1.alternateFunction(7);
    u2tx1.mode(Mode::ALTERNATE);

    USART usart(USART2, 115200);
    VN100 sensor{usart, 115200, VN100::CRCOptions::CRC_ENABLE_16};

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

    if (!sensor.start())
    {
        printf("Unable to start the sampling thread\n");
        return 0;
    }

    printf("Sensor sampling thread started!\n");

    // Sample and print 100 samples
    for (int i = 0; i < 100; i++)
    {
        sensor.sample();
        sample = sensor.getLastSample();
        printf("acc: %" PRIu64 ", %.3f, %.3f, %.3f\n",
               sample.accelerationTimestamp, sample.accelerationX,
               sample.accelerationY, sample.accelerationZ);
        printf("ang: %.3f, %.3f, %.3f\n", sample.angularSpeedX,
               sample.angularSpeedY, sample.angularSpeedZ);

        sensor.sampleRaw();
        sampleRaw = sensor.getLastRawSample();
        printf("%s\n", sampleRaw.c_str());
        // Thread::sleep(100);
        printf("\n");
    }

    sensor.closeAndReset();
    printf("Sensor communication closed!\n");

    return 0;
}
