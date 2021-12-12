/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Author: Nuno Barcellos
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
#include <diagnostic/CpuMeter.h>
#include <drivers/BusTemplate.h>
#include <drivers/spi/SensorSpi.h>
#include <interfaces-impl/hwmapping.h>
#include <math/Stats.h>
#include <sensors/ADIS16405/ADIS16405.h>
#include <sensors/SensorSampler.h>

using namespace miosix;
using namespace miosix::interfaces;

// Reset pin
typedef Gpio<GPIOD_BASE, 5> rstPin;  // PD5 for the HomeoneBoard

// SPI1 binding to the sensor
typedef BusSPI<1, spi1::mosi, spi1::miso, spi1::sck> busSPI1;  // Create SPI1
typedef ProtocolSPI<busSPI1, miosix::sensors::adis16405::cs>
    spiADIS16405;  // La lego al Chip Select 1 per la IMU 1

int main()
{
    spiADIS16405::init();

    Thread::sleep(1000);
    ADIS16405<spiADIS16405, rstPin>* adis =
        new ADIS16405<spiADIS16405, rstPin>(adis->GYRO_FS_300);

    if (adis->init())
        printf("[ADIS16405] Init succeeded\n");
    else
        printf("[ADIS16405] Init failed\n");

    if (adis->selfTest())
        printf("[ADIS16405] Self test succeeded\n");
    else
        printf("[ADIS16405] Self test failed\n");

    SimpleSensorSampler sampler(250, 1);
    sampler.addSensor(adis, std::bind([&]() {}));

    StatsResult statResult;
    Stats stats;

    int counter = 0;

    while (true)
    {
        sampler.sampleAndCallback();

        stats.add(averageCpuUtilization());

        if (counter == 2500)
        {
            statResult = stats.getStats();
            printf("CPU usage: %f\n", statResult.mean);
            counter = 0;

            const Vec3* last_data = adis->accelDataPtr();
            printf("%f %f %f\n", last_data->getX(), last_data->getY(),
                   last_data->getZ());
        }
        counter++;

        Thread::sleep(100);
    }
}
