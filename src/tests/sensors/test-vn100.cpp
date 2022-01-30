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
#include <sensors/VN100/VN100.h>

using namespace miosix;
using namespace Boardcore;

int main()
{
    GpioPin tx(GPIOB_BASE, 6);
    GpioPin rx(GPIOB_BASE, 7);
    VN100Data sample;
    string sampleRaw;
    VN100 sensor{1, VN100::BaudRates::Baud_921600,
                 VN100::CRCOptions::CRC_ENABLE_16};

    tx.mode(Mode::ALTERNATE);
    rx.mode(Mode::ALTERNATE);

    tx.alternateFunction(7);
    rx.alternateFunction(7);

    // Let the sensor start up
    Thread::sleep(1000);

    if (!sensor.init())
    {
        printf("Error initializing the sensor!\n");
        return 0;
    }

    printf("Sensor init successful!\n");

    if (!sensor.selfTest())
    {
        printf("Error self test check!\n");
        return 0;
    }

    printf("Sensor self test successful!\n");

    // Sample and print 100 samples
    for (int i = 0; i < 100; i++)
    {
        /*sensor.sample();
        sample = sensor.getLastSample();
        //printf("%" PRIu64 ", %.3f, %.3f, %.3f\n",
        sample.accelerationTimestamp, sample.accelerationX,
        sample.accelerationY, sample.accelerationZ);
        //printf("%.3f, %.3f, %.3f\n", sample.angularVelocityX,
        sample.angularVelocityY, sample.angularVelocityZ);*/

        sensor.sampleRaw();
        sampleRaw = sensor.getLastRawSample();
        printf("%s\n", sampleRaw.c_str());
        // Thread::sleep(100);
    }

    sensor.closeAndReset();
    printf("Sensor communication closed!\n");

    return 0;
}
