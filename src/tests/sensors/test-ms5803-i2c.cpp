/* Copyright (c) 2023 Skyward Experimental Rocketry
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

#include <sensors/MS5803/MS5803I2C.h>

using namespace Boardcore;
using namespace miosix;

GpioPin scl(GPIOA_BASE, 8);
GpioPin sda(GPIOC_BASE, 9);

int main()
{
    I2C bus(I2C3, scl, sda);
    MS5803I2C ms5803(bus, 10);

    if (!ms5803.init())
    {
        printf("MS5803 Init failed\n");
    }

    if (!ms5803.selfTest())
    {
        printf("Self test failed\n");
    }

    while (true)
    {
        ms5803.sample();

        auto data = ms5803.getLastSample();
        printf("[%.2f]: %.2fPa %.2f°\n", data.pressureTimestamp / 1e6,
               data.pressure, data.temperature);

        Thread::sleep(50);  // 25Hz
    }
}
