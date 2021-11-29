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

/**
 * This test has been setup for the following configuration:
 *
 * SPI pheripheral 2 (SPI2) with /32 divider
 *
 * Pins (STM32F407 - BME280):
 *  PB12 (NSS)  - not connected (we use pin C1 as chip select)
 *  PB13 (SCK)  - SCK
 *  PB14 (MISO) - SDO
 *  PB15 (MOSI) - SDA
 *  PC1         - CBS
 *
 * The BME280 is powered by 3.3V
 *
 * In the developing test a function generator was used as variable source
 */

#include <Debug.h>
#include <drivers/spi/SPIDriver.h>
#include <drivers/timer/TimestampTimer.h>
#include <miosix.h>
#include <sensors/BME280/BME280.h>

using namespace miosix;

using namespace Boardcore;

GpioPin sckPin  = GpioPin(GPIOB_BASE, 13);
GpioPin misoPin = GpioPin(GPIOB_BASE, 14);
GpioPin mosiPin = GpioPin(GPIOB_BASE, 15);
GpioPin csPin   = GpioPin(GPIOC_BASE, 1);

void initBoard()
{
    // Enable SPI clock for SPI2 interface
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    // Alternate function configuration for SPI pins
    sckPin.mode(miosix::Mode::ALTERNATE);
    sckPin.alternateFunction(5);  // SPI function
    mosiPin.mode(miosix::Mode::ALTERNATE);
    mosiPin.alternateFunction(5);  // SPI function
    misoPin.mode(miosix::Mode::ALTERNATE);
    misoPin.alternateFunction(5);  // SPI function

    // Chip select pin as output starting high
    csPin.mode(miosix::Mode::OUTPUT);
    csPin.high();
}

int main()
{
    // Enable SPI clock and set gpios
    initBoard();

    TimestampTimer::enableTimestampTimer();

    // SPI configuration setup
    SPIBusConfig spiConfig;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_32;
    spiConfig.mode         = SPI::Mode::MODE_0;
    SPIBus spiBus(SPI2);
    SPISlave spiSlave(spiBus, csPin, spiConfig);

    // Device initialization
    BME280 bme280(spiSlave);

    bme280.init();

    // In practice the self test reads the who am i reagister, this is already
    // done in init()
    if (!bme280.selfTest())
    {
        TRACE("Self test failed!\n");

        return -1;
    }

    // Try forced mode
    TRACE("Forced mode\n");
    for (int i = 0; i < 10; i++)
    {
        bme280.setSensorMode(BME280::FORCED_MODE);

        miosix::Thread::sleep(bme280.getMaxMeasurementTime());

        bme280.sample();

        TRACE("temp: %.2f DegC\tpress: %.2f hPa\thumid: %.2f %%RH\n",
              bme280.getLastSample().temp, bme280.getLastSample().press,
              bme280.getLastSample().humid);

        miosix::Thread::sleep(1000);
    }

    TRACE("Normal mode\n");
    bme280.setSensorMode(BME280::NORMAL_MODE);
    while (true)
    {
        bme280.sample();

        TRACE("temp: %.2f DegC\tpress: %.2f hPa\thumid: %.2f %%RH\n",
              bme280.getLastSample().temp, bme280.getLastSample().press,
              bme280.getLastSample().humid);

        miosix::Thread::sleep(40);  // 25Hz
    }
}