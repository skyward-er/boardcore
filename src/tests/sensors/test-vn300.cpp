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

    USART usart(USART6, 115200, 1000);
    VN300 sensor{usart, 115200, VN300::CRCOptions::CRC_ENABLE_8};

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
    for (int i = 0; i < 10; i++)
    {
        sensor.sample();
        sample = sensor.getLastSample();
        printf("acc: %" PRIu64 ", %.3f, %.3f, %.3f\n",
               sample.accelerationTimestamp, sample.accelerationX,
               sample.accelerationY, sample.accelerationZ);
        printf("ang: %.3f, %.3f, %.3f\n", sample.angularSpeedX,
               sample.angularSpeedY, sample.angularSpeedZ);
        printf("ins: %.3f, %.3f, %.3f\n", sample.yaw, sample.pitch,
               sample.roll);

        //sensor.sampleRaw();
        //sampleRaw = sensor.getLastRawSample();
        //printf("%s\n", sampleRaw.c_str());
        //printf("\n");
    }

    for (int j = 0; j < 10; j++)
    {
        uint64_t time_start = 0, time_end = 0;
        for (int i = 0; i < 10; i++)
        {   
            time_start = 0;
            time_end   = 0;
            time_start = TimestampTimer::getTimestamp();
            sensor.sample();
            sample   = sensor.getLastSample();
            time_end = TimestampTimer::getTimestamp();
            printf("Sample %i done in %" PRIu64 " microseconds\n", i,
                   (time_end - time_start));
            printf("acc: %" PRIu64 ", %.3f, %.3f, %.3f\n",
                   sample.accelerationTimestamp, sample.accelerationX,
                   sample.accelerationY, sample.accelerationZ);
            printf("ang: %.3f, %.3f, %.3f\n", sample.angularSpeedX,
                   sample.angularSpeedY, sample.angularSpeedZ);
            printf("ins: %.3f, %.3f, %.3f\n", sample.yaw, sample.pitch,
                   sample.roll);
            //Thread::sleep(20);
        }
        // = TimestampTimer::getTimestamp();
        // printf("Run %i done in %" PRIu64 " microseconds\n", j, (time_end -
        // time_start));
    }

   // sensor.closeAndReset();
    printf("Sensor communication closed!\n");

    return 0;
}
