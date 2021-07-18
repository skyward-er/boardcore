/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Luca Conterio
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
#include <drivers/serial.h>
#include <fcntl.h>
#include <filesystem/file_access.h>

using namespace miosix;

int main()
{
    // Open GPS serial on USART2 (PA2, PA3)
    intrusive_ref_ptr<DevFs> devFs = FilesystemManager::instance().getDevFs();
    devFs->addDevice("gps",
                     intrusive_ref_ptr<Device>(new STM32Serial(2, 38400)));

    TimestampTimer::enableTimestampTimer();

    // Keep GPS baud rate at default for easier testing
    UbloxGPS gps(38400);
    UbloxGPSData dataGPS;

    printf("init gps: %d\n", gps.init());
    Thread::sleep(200);
    gps.start();
    printf("selftest gps: %d\n", gps.selfTest());

    gps.sendSBASMessage();

    while (1)
    {
        Thread::sleep(2000);
        gps.sample();

        dataGPS = gps.getLastSample();
        printf(
            "timestamp: %.3f, fix: %d t: %lld lat: %f lon: %f height: %f nsat: "
            "%d speed: %f "
            "velN: %f velE: "
            "%f track %f\n",
            (float)dataGPS.gps_timestamp / 1000000, dataGPS.fix,
            dataGPS.gps_timestamp, dataGPS.latitude, dataGPS.longitude,
            dataGPS.height, dataGPS.num_satellites, dataGPS.speed,
            dataGPS.velocity_north, dataGPS.velocity_east, dataGPS.track);
    }
}
