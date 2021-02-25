/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Authors: Riccardo Musso
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#define BIAS_CALIBRATION_TEST 1

#include <Common.h>
#include <drivers/spi/SPIDriver.h>
#include <miosix.h>

#include "sensors/LIS3DSH/LIS3DSH.h"
#include "sensors/calibration/BiasCalibration.h"

using namespace miosix;

int main()
{
    GpioPin cs(GPIOE_BASE, 3), miso(GPIOA_BASE, 6), mosi(GPIOA_BASE, 7),
        clk(GPIOA_BASE, 5);
    SPIBus bus(SPI1);

    {
        FastInterruptDisableLock dLock;

        /* Enable SPI1 */
        RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

        cs.mode(Mode::OUTPUT);
        cs.high();

        clk.mode(Mode::ALTERNATE);
        clk.alternateFunction(5);

        miso.mode(Mode::ALTERNATE);
        miso.alternateFunction(5);

        mosi.mode(Mode::ALTERNATE);
        mosi.alternateFunction(5);
    }

    LIS3DSH sensor(bus, cs, sensor.ODR_100_HZ, sensor.UPDATE_AFTER_READ_MODE,
                   sensor.FULL_SCALE_4G);
    LIS3DSHData data;

    if (!sensor.init())
    {
        TRACE("Sensor error!");
        return -1;
    }

#ifdef BIAS_CALIBRATION_TEST
    BiasCalibration<AccelerometerData> model;

    for (int i = 0; i < 5; i++)
    {
        sensor.sample();
        data = sensor.getData();
        model.feed(data, AxisOrthoOrientation(Orientation::POSITIVE_X,
                                              Orientation::POSITIVE_Y));
    }

#endif

    ValuesCorrector<AccelerometerData>* corrector = model.computeResult();

    while (1)
    {

        TRACE("Sampling..\n");

        data = sensor.getData();
        TRACE("UNCORRECTED values:\t\tx: %f\ty: %f\tz: %f\n", data.accel_x,
              data.accel_y, data.accel_z);

        AccelerometerData data2 = corrector->correct(data);

        TRACE("CORRECTED values:\t\tx: %f\ty: %f\tz: %f\n", data2.accel_x,
              data2.accel_y, data2.accel_z);

        Thread::sleep(500);
    }
    return 0;
}

