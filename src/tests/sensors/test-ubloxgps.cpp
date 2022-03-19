/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Authors: Davide Bonomini, Davide Mor, Alberto Nidasio, Damiano Amatruda
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

#include <sensors/UbloxGPS/UbloxGPS.h>
#include <utils/Debug.h>

using namespace miosix;
using namespace Boardcore;

int main()
{
    SPIBus spiBus(SPI1);
    GpioPin spiCs(GPIOA_BASE, 3);
    GpioPin spiSck(GPIOA_BASE, 5);
    GpioPin spiMiso(GPIOA_BASE, 6);
    GpioPin spiMosi(GPIOA_BASE, 7);

    spiCs.mode(Mode::OUTPUT);
    spiCs.high();
    spiSck.mode(Mode::ALTERNATE);
    spiSck.alternateFunction(5);
    spiMiso.mode(Mode::ALTERNATE);
    spiMiso.alternateFunction(5);
    spiMosi.mode(Mode::ALTERNATE);
    spiMosi.alternateFunction(5);

    UbloxGPS sensor{spiBus, spiCs};

    TRACE("Initializing UbloxGPS...\n");

    if (!sensor.init())
    {
        TRACE("Init failed! (code: %d)\n", sensor.getLastError());
        return 1;
    }

    while (true)
    {
        sensor.sample();
        GPSData sample __attribute__((unused)) = sensor.getLastSample();

        TRACE(
            "timestamp: %4.3f, lat: %f, lon: %f, height: %4.1f, velN: %3.2f, "
            "velE: %3.2f, velD: %3.2f, speed: %3.2f, track %3.1f, pDOP: %f, "
            "nsat: %2d, fix: %01d\n",
            (float)sample.gpsTimestamp / 1000000, sample.latitude,
            sample.longitude, sample.height, sample.velocityNorth,
            sample.velocityEast, sample.velocityDown, sample.speed,
            sample.track, sample.positionDOP, sample.satellites, sample.fix);

        Thread::sleep(1000);
    }
}
