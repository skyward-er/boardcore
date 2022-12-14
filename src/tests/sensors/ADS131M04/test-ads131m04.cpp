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
#include <sensors/ADS131M04/ADS131M04.h>

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

    // SPI configuration setup
    SPIBus spiBus(SPI4);
    SPIBusConfig spiConfig = {};
    spiConfig.mode         = SPI::Mode::MODE_1;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_64;
    SPISlave spiSlave(spiBus, csPin, spiConfig);

    // Device initialization
    ADS131M04 ads131(spiSlave);

    // Initialize the device
    ads131.init();
    ads131.enableGlobalChopMode();
    ads131.setOversamplingRatio(ADS131M04::OversamplingRatio::OSR_16256);
    ads131.calibrateOffset();

    while (true)
    {
        ads131.sample();

        ADS131M04Data data = ads131.getLastSample();

        printf("% 2.5f\t% 2.5f\t% 2.5f\t% 2.5f\n", data.voltage[0],
               data.voltage[1], data.voltage[2], data.voltage[3]);

        delayMs(50);
    }
}
