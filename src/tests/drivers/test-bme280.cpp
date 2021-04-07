/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Alberto Nidasio
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
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
 * Pins (STM32F407 - ADS1118):
 *  PB12 (NSS)  - NC (we use pin C1 as chip select)
 *  PB13 (SCK)  - SCK
 *  PB14 (MISO) - DOUT
 *  PB15 (MOSI) - DIN
 *  PC1         - CS
 *
 * The ADS1118's input channels can be connected as follow:
 *  AIN2 - GND
 *  AIN3 - VCC (3V)
 *
 * In the developing test a function generator was used as variable source
 */

#include <Debug.h>
#include <drivers/spi/SPIDriver.h>
#include <miosix.h>
#include <sensors/BME280/BME280.h>

#include "TimestampTimer.h"

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
    spiConfig.clock_div = SPIClockDivider::DIV32;
    spiConfig.mode      = SPIMode::MODE0;
    SPIBus spiBus(SPI2);
    SPISlave spiSlave(spiBus, csPin, spiConfig);

    // Device initialization
    BME280 bme280(spiSlave);

    bool ret = bme280.init();

    TRACE("size of BME280Comp: %d\n", sizeof(BME280::BME280Config));

    TRACE("ret: %d\n", ret);
    TRACE("error: %d\n", bme280.getLastError());
}