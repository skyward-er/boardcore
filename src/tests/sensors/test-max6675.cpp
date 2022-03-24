/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

#include <drivers/timer/TimestampTimer.h>
#include <miosix.h>
#include <sensors/MAX6675/MAX6675.h>

using namespace miosix;
using namespace Boardcore;

GpioPin sckPin  = GpioPin(GPIOE_BASE, 2);
GpioPin misoPin = GpioPin(GPIOE_BASE, 5);
GpioPin mosiPin = GpioPin(GPIOE_BASE, 6);
GpioPin csPin   = GpioPin(GPIOE_BASE, 4);

void initBoard()
{
    // Setup gpio pins
    csPin.mode(Mode::OUTPUT);
    csPin.high();
    sckPin.mode(Mode::ALTERNATE);
    sckPin.alternateFunction(5);
    misoPin.mode(Mode::ALTERNATE);
    misoPin.alternateFunction(5);
    mosiPin.mode(Mode::ALTERNATE);
    mosiPin.alternateFunction(5);
}

int main()
{
    // Enable SPI clock and set gpios
    initBoard();

    SPIBus spiBus(SPI4);
    MAX6675 sensor{spiBus, csPin};

    printf("Starting process verification!\n");

    if (!sensor.selfTest())
    {
        printf("Sensor self test failed!\n");
    }

    while (true)
    {
        sensor.sample();
        TemperatureData sample = sensor.getLastSample();

        printf("%.2f\n", sample.temperature);

        Thread::sleep(500);
    }
}
