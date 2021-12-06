/* Copyright (c) 2019 Skyward Experimental Rocketry
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
#include <sensors/LSM6DS3H/LSM6DS3H.h>
#include <sensors/SensorSampler.h>

using namespace miosix;
using namespace miosix::interfaces;

/* SPI1 binding to the sensor */
typedef BusSPI<1, spi1::mosi, spi1::miso, spi1::sck> busSPI1;
typedef ProtocolSPI<busSPI1, sensors::lsm6ds3h::cs> spiLSM6DS3H0_a;

int main()
{
    SimpleSensorSampler sampler(250, 1);
    spiLSM6DS3H0_a::init();

    LSM6DS3H<spiLSM6DS3H0_a>* lsm6ds3h = new LSM6DS3H<spiLSM6DS3H0_a>(3, 3);

    if (lsm6ds3h->init())
    {
        printf("[LSM6DS3H] Init succeeded\n");
        sampler.addSensor(lsm6ds3h, std::bind([&]() {}));
    }
    else
    {
        printf("[LSM6DS3H] Init failed\n");

        while (!lsm6ds3h->init())
        {
            printf("[LSM6DS3H] Init failed\n");
            Thread::sleep(1000);
        }
    }

    while (true)
    {
        sampler.sampleAndCallback();

        // const Vec3* last_data = lsm6ds3h->gyroDataPtr();
        // printf("%f %f %f\n", last_data->getX(), last_data->getY(),
        //        last_data->getZ());

        const Vec3* last_data = lsm6ds3h->accelDataPtr();
        printf("%f %f %f\n", last_data->getX(), last_data->getY(),
               last_data->getZ());

        // const float* last_temp = lsm6ds3h->tempDataPtr();
        // printf("temp: %f\n", *last_temp);

        Thread::sleep(100);
    }
}