/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Davide Bonomini, Davide Mor, Alberto Nidasio
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
#include <miosix.h>
#include <sensors/UBXGPS/UBXGPSSerial.h>
#include <utils/Debug.h>

#include <cstdio>

using namespace Boardcore;
using namespace miosix;

typedef miosix::Gpio<GPIOD_BASE, 5> u2tx2;

#define RATE 1

int main()
{
    u2tx2::mode(Mode::ALTERNATE);
    u2tx2::alternateFunction(7);

    // Keep GPS baud rate at default for easier testing
    UBXGPSSerial gps(38400, RATE, 2, "gps", 9600);
    UBXGPSData data;
    printf("Gps allocated\n");

    // Init the gps
    if (gps.init())
        printf("Successful gps initialization\n");
    else
        printf("Failed gps initialization\n");

    // Perform the self test
    if (gps.selfTest())
        printf("Successful gps self test\n");
    else
        printf("Failed gps self test\n");

    // Start the gps thread
    gps.start();
    printf("Gps started\n");

    while (true)
    {
        // Give time to the thread
        Thread::sleep(1000 / RATE);

        // Sample
        gps.sample();
        data = gps.getLastSample();

        // Print out the latest sample
        printf(
            "[%.2f] lat: % f lon: % f height: %4.1f velN: % 3.2f velE: % 3.2f "
            "velD: % 3.2f speed: %3.2f track: %.2f posDOP: %.2f nSat: %2d fix: "
            "%01d\n",
            data.gpsTimestamp / 1e6, data.latitude, data.longitude, data.height,
            data.velocityNorth, data.velocityEast, data.velocityDown,
            data.speed, data.track, data.positionDOP, data.satellites,
            data.fix);
    }
}
