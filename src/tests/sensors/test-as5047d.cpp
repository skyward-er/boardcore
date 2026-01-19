/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Raul Radu
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
#include <sensors/AS5047D/AS5047DSPI.h>

using namespace miosix;
using namespace Boardcore;

GpioPin sckPin  = GpioPin(GPIOA_BASE, 5);
GpioPin misoPin = GpioPin(GPIOA_BASE, 6);
GpioPin mosiPin = GpioPin(GPIOA_BASE, 7);
GpioPin csPin   = GpioPin(GPIOA_BASE, 15);

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

    AS5047DSPIConfig config;
    config.dataType    = AS5047DDefs::DataSelect::DAECANG;
    config.daecEnabled = AS5047DDefs::DAECStatus::DAEC_ON;

    // Device initialization
    SPIBus spiBus(SPI1);
    AS5047DSPI as5047d(spiBus, csPin, config);
    delayMs(10);
    // as5047d.init();
    if (!as5047d.init())
    {
        printf("Failed initialization\n");
        for (;;)
        {
        }
    }

    printf("Now performing self test...\n");
    if (as5047d.selfTest())
        printf("Self test succeeded\n");
    else
        printf("Self test failed!\n");

    while (true)
    {
        as5047d.sample();

        AS5047DData data = as5047d.getLastSample();

        printf("timestamp:%lld,angle:%f\r\n", data.timestamp, data.angle);
        delayMs(10);
    }
}
