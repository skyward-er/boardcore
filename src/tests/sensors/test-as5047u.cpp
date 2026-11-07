/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Aldas Lenkšas
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
#include <miosix.h>
#include <sensors/AS5047U/AS5047U.h>

#include <iostream>

using namespace miosix;
using namespace Boardcore;

GpioPin sckPin  = GpioPin(GPIOA_BASE, 5);
GpioPin misoPin = GpioPin(GPIOA_BASE, 6);
GpioPin mosiPin = GpioPin(GPIOA_BASE, 7);
GpioPin csPin   = GpioPin(GPIOA_BASE, 4);

void initBoard()
{
    // Setup gpio pins
    sckPin.mode(miosix::Mode::ALTERNATE);
    sckPin.alternateFunction(5);
    misoPin.mode(miosix::Mode::ALTERNATE);
    misoPin.alternateFunction(5);
    mosiPin.mode(miosix::Mode::ALTERNATE);
    mosiPin.alternateFunction(5);
    csPin.mode(miosix::Mode::OUTPUT);
    csPin.high();
}

int main()
{
    std::cout << "START" << std::endl;

    // Enable SPI clock and set gpios
    initBoard();

    // Sensor config: leaving default values
    AS5047UConfig config{};

    // SPI and sensor config
    SPIBus spiBus(SPI1);
    AS5047U<AS5047UFrames::SPIFrame24> sensor(spiBus, csPin, config);

    if (!sensor.init())
    {
        std::cout << "SENSOR INIT FAILED" << std::endl;
        return 0;
    }

    std::cout << "SENSOR INIT SUCCESSFUL";

    if (!sensor.resetAngleZero())
        std::cout << "RESET ANGLE ZERO FAILED" << std::endl;
    std::cout << "RESETTED ANGLE ZERO" << std::endl;

    sensor.sample();  // the first sample is invalid

    while (true)
    {
        sensor.sample();
        auto data = sensor.getLastSample();
        std::cout << "SAMPLE angle: " << data.angle << std::endl;

        delayMs(100);
    }
}
