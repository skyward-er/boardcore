/* Copyright (c) 2022 Skyward Experimental Rocketry
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

#include <drivers/timer/TimestampTimer.h>
#include <miosix.h>
#include <sensors/MAX31856/MAX31856.h>

using namespace miosix;
using namespace Boardcore;

GpioPin sckPin  = GpioPin(GPIOB_BASE, 3);
GpioPin misoPin = GpioPin(GPIOB_BASE, 4);
GpioPin mosiPin = GpioPin(GPIOD_BASE, 6);
GpioPin csPin   = GpioPin(GPIOD_BASE, 4);

int main()
{
    // Setup gpio pins
    sckPin.mode(Mode::ALTERNATE);
    sckPin.alternateFunction(6);
    misoPin.mode(Mode::ALTERNATE);
    misoPin.alternateFunction(6);
    mosiPin.mode(Mode::ALTERNATE);
    mosiPin.alternateFunction(5);
    csPin.mode(Mode::OUTPUT);
    csPin.high();

    SPIBus spiBus(SPI3);
    MAX31856 sensor{spiBus, csPin};

    printf("Starting process verification!\n");

    sensor.init();

    if (!sensor.selfTest())
    {
        printf("The thermocouple is not connected\n");
    }
    else
    {
        printf("The thermocouple is connected\n");
    }

    // Wait for one sample to be made
    Thread::sleep(100);

    // Set cold junction offset
    // This should set the cold junction temperature to 25°
    sensor.sample();
    auto sample = sensor.getLastSample();
    sensor.setColdJunctionOffset(sample.coldJunctionTemperature - 25);
    printf("Offset: %f\n", sample.coldJunctionTemperature - 25);

    while (true)
    {
        sensor.sample();
        sample = sensor.getLastSample();

        printf("[%.2f] %f %f\n", sample.temperatureTimestamp / 1e6,
               sample.temperature, sample.coldJunctionTemperature);

        Thread::sleep(100);
    }
}
