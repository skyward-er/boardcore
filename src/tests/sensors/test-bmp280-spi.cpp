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
#include <sensors/BMP280/BMP280.h>
#include <utils/Debug.h>

using namespace miosix;
using namespace Boardcore;

GpioPin sckPin  = GpioPin(GPIOA_BASE, 5);
GpioPin misoPin = GpioPin(GPIOB_BASE, 4);
GpioPin mosiPin = GpioPin(GPIOA_BASE, 7);
GpioPin csPin   = GpioPin(GPIOB_BASE, 2);

void initBoard()
{
    // Alternate function configuration for SPI pins
    sckPin.mode(Mode::ALTERNATE);
    sckPin.alternateFunction(5);  // SPI function
    mosiPin.mode(Mode::ALTERNATE);
    mosiPin.alternateFunction(5);  // SPI function
    misoPin.mode(Mode::ALTERNATE);
    misoPin.alternateFunction(5);  // SPI function

    // Chip select pin as output starting high
    csPin.mode(Mode::OUTPUT);
    csPin.high();
}

int main()
{
    // Enable SPI clock and set gpios
    initBoard();

    // SPI configuration setup
    SPIBusConfig spiConfig;
    // spiConfig.clockDivider = SPI::ClockDivider::DIV_32;
    spiConfig.mode = SPI::Mode::MODE_0;
    SPIBus spiBus(SPI1);
    SPISlave spiSlave(spiBus, csPin, spiConfig);

    // Device initialization
    BMP280 bmp280(spiSlave);

    bmp280.init();

    // In practice the self test reads the who am i reagister, this is already
    // done in init()
    if (!bmp280.selfTest())
    {
        TRACE("Self test failed!\n");

        return -1;
    }

    // Try forced mode
    TRACE("Forced mode\n");
    for (int i = 0; i < 10; i++)
    {
        bmp280.setSensorMode(BMP280::FORCED_MODE);

        Thread::sleep(bmp280.getMaxMeasurementTime());

        bmp280.sample();

        TRACE("temp: %.2f DegC\tpress: %.2f hPa\n",
              bmp280.getLastSample().temperature,
              bmp280.getLastSample().pressure);

        Thread::sleep(1000);
    }

    TRACE("Normal mode\n");
    bmp280.setSensorMode(BMP280::NORMAL_MODE);
    while (true)
    {
        bmp280.sample();

        TRACE("temp: %.2f DegC\tpress: %.2f Pa\thumid: %.2f %%RH\n",
              bmp280.getLastSample().temperature,
              bmp280.getLastSample().pressure);

        Thread::sleep(50);  // 25Hz
    }
}
