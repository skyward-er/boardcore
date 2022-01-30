/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Riccardo Musso
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

/*  ACCELEROMETER calibration: please set the chosen one to 1  */
#define BIAS_CALIBRATION_LOAD_TEST 1
#define SIX_PARAMETER_CALIBRATION_LOAD_TEST 0
#define TWELVE_PARAMETER_CALIBRATION_LOAD_TEST 0

/* Expressed in Hertz: valid values: 1 <= frequency <= 1000 */
#define SAMPLE_FREQUENCY_LOAD_TEST 1000

#include <diagnostic/CpuMeter.h>
#include <drivers/spi/SPIDriver.h>
#include <miosix.h>
#include <sensors/LIS3DSH/LIS3DSH.h>
#include <sensors/calibration/BiasCalibration.h>
#include <sensors/calibration/HardIronCalibration.h>
#include <sensors/calibration/SixParameterCalibration.h>
#include <sensors/calibration/SoftIronCalibration.h>
#include <sensors/calibration/TwelveParameterCalibration.h>
#include <utils/Debug.h>

using namespace Boardcore;
using namespace miosix;

/* using volatile so that the compiler won't optimize out the variable */
volatile AccelerometerData testData;

int main()
{
#if BIAS_CALIBRATION_LOAD_TEST
    BiasCorrector<AccelerometerData> corrector;
    TRACE("Using BIAS calibration model.\n");
#endif

#if SIX_PARAMETER_CALIBRATION_LOAD_TEST
    SixParameterCorrector<AccelerometerData> corrector;
    TRACE("Using SIX-PARAMETER calibration model.\n");
#endif

#if TWELVE_PARAMETER_CALIBRATION_LOAD_TEST
    TwelveParameterCorrector<AccelerometerData> corrector;
    TRACE("Using TWELVE-PARAMETER calibration model.\n");
#endif

    SPIBus bus(SPI1);

    GpioPin spiSck(GPIOA_BASE, 5);
    GpioPin spiMiso(GPIOA_BASE, 6);
    GpioPin spiMosi(GPIOA_BASE, 7);
    GpioPin cs(GPIOE_BASE, 3);

    {
        miosix::FastInterruptDisableLock dLock;

        RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;  // Enable SPI1 bus

        spiSck.mode(miosix::Mode::ALTERNATE);
        spiSck.alternateFunction(5);
        spiMiso.mode(miosix::Mode::ALTERNATE);
        spiMiso.alternateFunction(5);
        spiMosi.mode(miosix::Mode::ALTERNATE);
        spiMosi.alternateFunction(5);

        cs.mode(miosix::Mode::OUTPUT);
    }
    cs.high();

    LIS3DSH sensor(bus, cs, sensor.ODR_100_HZ, sensor.UPDATE_AFTER_READ_MODE,
                   sensor.FULL_SCALE_4G);
    sensor.init();

    constexpr unsigned sleepMillis = 1000 / SAMPLE_FREQUENCY_LOAD_TEST;
    unsigned elapsed               = 0;

    while (1)
    {
        Thread::sleep(sleepMillis);

        elapsed += sleepMillis;
        sensor.sample();

        AccelerometerData tmp = corrector.correct(sensor.getLastSample());

        testData.accelerationX = tmp.accelerationX;
        testData.accelerationY = tmp.accelerationY;
        testData.accelerationZ = tmp.accelerationZ;

        if (elapsed > 500)
        {
            elapsed = 0;
            TRACE("Average CPU usage: %f %%\n", averageCpuUtilization());
        }
    }
}
