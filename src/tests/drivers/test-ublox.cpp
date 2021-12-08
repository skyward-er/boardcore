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

#include <Common.h>
#include <drivers/gps/ublox/UbloxGPS.h>

#include <cstdio>

#include "TimestampTimer.h"

using namespace Boardcore;
using namespace miosix;

#define RATE 4

int main()
{
    TimestampTimer::enableTimestampTimer();

    printf("Welcome to the ublox test\n");

    // Keep GPS baud rate at default for easier testing
    UbloxGPS gps(921600, RATE, 2, "gps", 38400);
    UbloxGPSData dataGPS;
    printf("Gps allocated\n");

    // Init the gps
    if (gps.init())
    {
        printf("Successful gps initialization\n");
    }
    else
    {
        printf("Failed gps initialization\n");
    }

    // Perform the selftest
    if (gps.selfTest())
    {
        printf("Successful gps selftest\n");
    }
    else
    {
        printf("Failed gps selftest\n");
    }

    // Start the gps thread
    gps.start();
    printf("Gps started\n");

    while (true)
    {
        // Give time to the thread
        Thread::sleep(1000 / RATE);

        // Sample
        gps.sample();
        dataGPS = gps.getLastSample();

        // Print out the latest sample
        TRACE(
            "[gps] timestamp: % 4.3f, fix: %01d lat: % f lon: % f "
            "height: %4.1f nsat: %2d speed: %3.2f velN: % 3.2f velE: % 3.2f "
            "track %3.1f\n",
            (float)dataGPS.gps_timestamp / 1000000, dataGPS.fix,
            dataGPS.latitude, dataGPS.longitude, dataGPS.height,
            dataGPS.num_satellites, dataGPS.speed, dataGPS.velocity_north,
            dataGPS.velocity_east, dataGPS.track);
    }
}
