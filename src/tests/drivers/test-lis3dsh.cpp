/* Copyright (c) 2020 Skyward Experimental Rocketry
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

#include <drivers/spi/SPIDriver.h>
#include <miosix.h>
#include <sensors/LIS3DSH/LIS3DSH.h>

#include "Common.h"

using namespace Boardcore;
using namespace std;
using namespace miosix;

SPIBus bus(SPI1);

GpioPin spi_sck(GPIOA_BASE, 5);
GpioPin spi_miso(GPIOA_BASE, 6);
GpioPin spi_mosi(GPIOA_BASE, 7);
GpioPin cs(GPIOE_BASE, 3);

int main()
{
    {
        miosix::FastInterruptDisableLock dLock;

        RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;  // Enable SPI1 bus

        spi_sck.mode(miosix::Mode::ALTERNATE);
        spi_sck.alternateFunction(5);
        spi_miso.mode(miosix::Mode::ALTERNATE);
        spi_miso.alternateFunction(5);
        spi_mosi.mode(miosix::Mode::ALTERNATE);
        spi_mosi.alternateFunction(5);

        cs.mode(miosix::Mode::OUTPUT);
    }
    cs.high();

    TimestampTimer::enableTimestampTimer();

    LIS3DSH sensor(bus, cs, sensor.ODR_100_HZ, sensor.UPDATE_AFTER_READ_MODE,
                   sensor.FULL_SCALE_4G);

    LIS3DSHData data;

    // sensor not initialized, should give the error
    sensor.sample();
    if (sensor.getLastError() != SensorErrors::NOT_INIT)
    {
        printf("\nTEST FAILED: sensor not initialized \n");
        return -1;
    }

    bool initialized = false;
    // initialize imu
    if (!sensor.init())
    {
        if (sensor.getLastError() == SensorErrors::INVALID_WHOAMI)
        {
            printf("TEST FAILED: invalid WHO_AM_I value, init failed \n");
        }
        else
        {
            printf("TEST FAILED: init failed \n");
        }
        return -1;
    }
    initialized = true;

    // check if the sensor is properly working
    if (!sensor.selfTest())
    {
        printf("\nTEST FAILED: self-test failed \n");
        return -1;
    }

    // if sensor already inizialized, init() should return false
    if (initialized)
    {
        if (sensor.init())
        {
            printf("\nTEST FAILED: sensor is already initialized \n");
            return -1;
        }
    }

    Thread::sleep(500);

    // sample some data from the sensor
    for (int i = 0; i < 5; i++)
    {
        // sensor intitialized, should return error if no new data exist
        sensor.sample();

        if (sensor.getLastError() == SensorErrors::NO_NEW_DATA)
        {
            printf("\nWarning: no new data to be read \n");
        }

        data = sensor.getLastSample();

        printf("\nTimestamp: %llu \n", data.accel_timestamp);
        printf("Acc: x: %f | y: %f | z: %f \n", data.accel_x, data.accel_y,
               data.accel_z);
        printf("Temp: %.2f C \n", data.temp);

        Thread::sleep(200);
    }

    printf("\nLIS3DSH TEST OK ! \n");

    return 0;
}