/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include <miosix.h>
#include <sensors/BME280/BME280I2C.h>

using namespace miosix;
using namespace Boardcore;

GpioPin scl(GPIOA_BASE, 8);
GpioPin sda(GPIOC_BASE, 9);

int main()
{
    I2C bus(I2C3, scl, sda);
    BME280I2C bme280(bus);
    bme280.init();

    if (!bme280.selfTest())
        printf("Self test failed!\n");

    printf("Forced mode\n");
    for (int i = 0; i < 10; i++)
    {
        bme280.setSensorMode(BME280I2C::FORCED_MODE);

        Thread::sleep(bme280.getMaxMeasurementTime());

        bme280.sample();

        printf("temp: %.2f DegC\tpress: %.2f hPa\thumid: %.2f %%RH\n",
               bme280.getLastSample().temperature,
               bme280.getLastSample().pressure.value(),
               bme280.getLastSample().humidity);

        Thread::sleep(1000);
    }

    printf("Normal mode\n");
    bme280.setSensorMode(BME280I2C::NORMAL_MODE);
    while (true)
    {
        bme280.sample();

        printf("temp: %.2f DegC\tpress: %.2f Pa\thumid: %.2f %%RH\n",
               bme280.getLastSample().temperature,
               bme280.getLastSample().pressure.value(),
               bme280.getLastSample().humidity);

        Thread::sleep(50);  // 25Hz
    }
}
