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

#include <drivers/spi/SPIDriver.h>
#include <drivers/timer/TimestampTimer.h>
#include <miosix.h>
#include <sensors/ADS131M08/ADS131M08.h>

using namespace miosix;
using namespace Boardcore;

GpioPin sckPin  = GpioPin(GPIOA_BASE, 5);
GpioPin misoPin = GpioPin(GPIOA_BASE, 6);
GpioPin mosiPin = GpioPin(GPIOA_BASE, 7);
GpioPin csPin   = GpioPin(GPIOA_BASE, 3);

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

    // SPI configuration setup
    SPIBus spiBus(SPI1);
    SPISlave spiSlave(spiBus, csPin, ADS131M08::getDefaultSPIConfig());

    // Device initialization
    ADS131M08 ads131(spiSlave);

    // Initialize the device
    ads131.init();
    ads131.enableGlobalChopMode();
    ads131.setOversamplingRatio(ADS131M08::OversamplingRatio::OSR_16256);
    ads131.calibrateOffset();

    while (true)
    {
        ads131.sample();

        ADS131M08Data data = ads131.getLastSample();

        printf(
            "% 2.5f\t% 2.5f\t% 2.5f\t% 2.5f\t% 2.5f\t% 2.5f\t% 2.5f\t% 2.5f\n",
            data.voltage[0], data.voltage[1], data.voltage[2], data.voltage[3],
            data.voltage[4], data.voltage[5], data.voltage[6], data.voltage[7]);

        delayMs(50);
    }
}
