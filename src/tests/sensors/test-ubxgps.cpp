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

#include <sensors/UBXGPS/UBXGPS.h>
#include <utils/Debug.h>

using namespace miosix;
using namespace Boardcore;

int main()
{
#ifdef _BOARD_STM32F429ZI_SKYWARD_DEATHST_X
    SPIBus spiBus(SPI2);
    GpioPin spiCs(GPIOG_BASE, 3);
    GpioPin spiSck(GPIOB_BASE, 13);
    GpioPin spiMiso(GPIOB_BASE, 14);
    GpioPin spiMosi(GPIOB_BASE, 15);
#else
    SPIBus spiBus(SPI1);
    GpioPin spiCs(GPIOE_BASE, 3);
    GpioPin spiSck(GPIOA_BASE, 5);
    GpioPin spiMiso(GPIOA_BASE, 6);
    GpioPin spiMosi(GPIOA_BASE, 7);
#endif

    spiCs.mode(Mode::OUTPUT);
    spiCs.high();
    spiSck.mode(Mode::ALTERNATE);
    spiSck.alternateFunction(5);
    spiMiso.mode(Mode::ALTERNATE);
    spiMiso.alternateFunction(5);
    spiMosi.mode(Mode::ALTERNATE);
    spiMosi.alternateFunction(5);

    UBXGPS gps{spiBus, spiCs};

    TRACE("Initializing UBXGPS...\n");

    while (!gps.init())
    {
        TRACE("Init failed! (code: %d)\n", gps.getLastError());

        TRACE("Retrying in 10 seconds...\n");
        miosix::Thread::sleep(10000);
    }

    while (true)
    {
        gps.sample();
        GPSData sample __attribute__((unused)) = gps.getLastSample();

        TRACE(
            "timestamp: %4.3f, lat: %f, lon: %f, height: %4.1f, velN: %3.2f, "
            "velE: %3.2f, velD: %3.2f, speed: %3.2f, track %3.1f, pDOP: %f, "
            "nsat: %2d, fix: %2d\n",
            (float)sample.gpsTimestamp / 1000000, sample.latitude,
            sample.longitude, sample.height, sample.velocityNorth,
            sample.velocityEast, sample.velocityDown, sample.speed,
            sample.track, sample.positionDOP, sample.satellites, sample.fix);

        Thread::sleep(1000 / gps.getSampleRate());
    }
}
