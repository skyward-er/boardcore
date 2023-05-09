/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Radu Raul
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
#include <drivers/timer/TimestampTimer.h>
#include <sensors/H3LIS331DL/H3LIS331DL.h>
#include <utils/Debug.h>

#include "miosix.h"

using namespace Boardcore;
using namespace miosix;

SPIBus bus(SPI1);

GpioPin spiSck(GPIOA_BASE, 5);
GpioPin spiMiso(GPIOA_BASE, 6);
GpioPin spiMosi(GPIOA_BASE, 7);
GpioPin cs(GPIOE_BASE, 4);

int main()
{
    Thread::sleep(1000);

    /**
     * I need to set the pin speed to Speed::_100MHz to solve a bug with
     * stm32f407vg boards that prevented a correct reading from SPI (in this
     * scenario)
     */
    spiSck.mode(miosix::Mode::ALTERNATE);
    spiSck.alternateFunction(5);
    spiSck.speed(Speed::_100MHz);

    spiMiso.mode(miosix::Mode::ALTERNATE);
    spiMiso.alternateFunction(5);
    spiMiso.speed(Speed::_100MHz);

    spiMosi.mode(miosix::Mode::ALTERNATE);
    spiMosi.alternateFunction(5);
    spiMosi.speed(Speed::_100MHz);

    cs.mode(miosix::Mode::OUTPUT);
    cs.high();

    H3LIS331DL sensor(bus, cs, H3LIS331DLDefs::OutputDataRate::ODR_50,
                      H3LIS331DLDefs::BlockDataUpdate::BDU_CONTINUOS_UPDATE,
                      H3LIS331DLDefs::FullScaleRange::FS_100);

    if (!sensor.init())
    {
        printf("Failed init!\n");
        if (sensor.getLastError() == SensorErrors::INVALID_WHOAMI)
        {
            printf("Invalid WHOAMI\n");
        }
        return -1;
    }

    H3LIS331DLData data;

    // Print out the CSV header
    printf(H3LIS331DLData::header().c_str());
    // sample some data from the sensor
    for (int i = 0; i < 255; i++)
    {
        // sensor intitialized, should return error if no new data exist
        sensor.sample();

        // if (sensor.getLastError() == SensorErrors::NO_NEW_DATA)
        // {
        //     printf("\nWarning: no new data to be read \n");
        // }

        data = sensor.getLastSample();

        printf("%llu,%f,%f,%f\n", data.accelerationTimestamp,
               data.accelerationX, data.accelerationY, data.accelerationZ);

        Thread::sleep(100);
    }

    return 0;
}
