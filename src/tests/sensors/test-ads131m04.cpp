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

#include <drivers/spi/SPIDriver.h>
#include <drivers/timer/TimestampTimer.h>
#include <miosix.h>
#include <sensors/ADS131M04/ADS131M04.h>

using namespace miosix;
using namespace Boardcore;

GpioPin sckPin  = GpioPin(GPIOA_BASE, 5);
GpioPin misoPin = GpioPin(GPIOA_BASE, 6);
GpioPin mosiPin = GpioPin(GPIOA_BASE, 7);
GpioPin csPin   = GpioPin(GPIOA_BASE, 4);

void initBoard()
{
    // Setup gpio pins
    sckPin.mode(Mode::ALTERNATE);
    sckPin.alternateFunction(5);
    misoPin.mode(Mode::ALTERNATE);
    misoPin.alternateFunction(5);
    mosiPin.mode(Mode::ALTERNATE);
    mosiPin.alternateFunction(5);
    csPin.mode(Mode::OUTPUT);
    csPin.high();
}

int main()
{
    // Enable SPI clock and set gpios
    initBoard();

    // ADC configuration
    ADS131M04::Config config{
        .oversamplingRatio     = ADS131M04Defs::OversamplingRatio::OSR_8192,
        .globalChopModeEnabled = true,
    };

    // Device initialization
    SPIBus spiBus(SPI1);
    ADS131M04 ads131(spiBus, csPin, SPIBusConfig(), config);

    ads131.init();

    printf("Now performing self test...\n");
    if (ads131.selfTest())
    {
        printf("Self test succeeded\n");
    }
    else
    {
        printf("Self test failed!\n");
    }

    // ads131.calibrateOffset(ADS131M04Defs::Channel::CHANNEL_0);
    // ads131.calibrateOffset(ADS131M04Defs::Channel::CHANNEL_1);
    // ads131.calibrateOffset(ADS131M04Defs::Channel::CHANNEL_2);
    // ads131.calibrateOffset(ADS131M04Defs::Channel::CHANNEL_3);

    while (true)
    {
        ads131.sample();

        ADS131M04Data data = ads131.getLastSample();

        printf("% 2.8f\t% 2.8f\t% 2.8f\t% 2.8f\n", data.voltage[0],
               data.voltage[1], data.voltage[2], data.voltage[3]);

        delayMs(50);
    }
}
