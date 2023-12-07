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
#include <sensors/ADS131M08/ADS131M08.h>

using namespace miosix;
using namespace Boardcore;

GpioPin sckPin  = GpioPin(GPIOE_BASE, 2);
GpioPin misoPin = GpioPin(GPIOE_BASE, 5);
GpioPin mosiPin = GpioPin(GPIOE_BASE, 6);
GpioPin csPin   = GpioPin(GPIOG_BASE, 10);

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
    ADS131M08::Config config{
        .oversamplingRatio     = ADS131M08Defs::OversamplingRatio::OSR_8192,
        .globalChopModeEnabled = true,
    };

    // Device initialization
    SPIBus spiBus(SPI4);
    ADS131M08 ads131(spiBus, csPin, SPIBusConfig(), config);

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

    ads131.calibrateOffset(ADS131M08Defs::Channel::CHANNEL_0);
    ads131.calibrateOffset(ADS131M08Defs::Channel::CHANNEL_1);
    ads131.calibrateOffset(ADS131M08Defs::Channel::CHANNEL_2);
    ads131.calibrateOffset(ADS131M08Defs::Channel::CHANNEL_3);
    ads131.calibrateOffset(ADS131M08Defs::Channel::CHANNEL_4);
    ads131.calibrateOffset(ADS131M08Defs::Channel::CHANNEL_5);
    ads131.calibrateOffset(ADS131M08Defs::Channel::CHANNEL_6);
    ads131.calibrateOffset(ADS131M08Defs::Channel::CHANNEL_7);

    while (true)
    {
        ads131.sample();

        ADS131M08Data data = ads131.getLastSample();

        printf(
            "% 2.8f\t% 2.8f\t% 2.8f\t% 2.8f\t% 2.8f\t% 2.8f\t% 2.8f\t% 2.8f\n",
            data.voltage[0], data.voltage[1], data.voltage[2], data.voltage[3],
            data.voltage[4], data.voltage[5], data.voltage[6], data.voltage[7]);

        delayMs(100);
    }
}
