/* Copyright (c) 2020 Skyward Experimental Rocketry
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
 * @file test-ads1118.cpp
 * @author Alberto Nidasio (alberto.nidasio@skywarder.eu)
 * @brief Tests the ads1118 adc
 * @version 1.0
 * @date 2020-11-19
 *
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
#include <drivers/adc/ADS1118/ADS1118.h>
#include <drivers/spi/SPIDriver.h>
#include <miosix.h>

#include "TimestampTimer.h"

GpioPin sckPin  = GpioPin(GPIOB_BASE, 13);
GpioPin misoPin = GpioPin(GPIOB_BASE, 14);
GpioPin mosiPin = GpioPin(GPIOB_BASE, 15);
GpioPin csPin   = GpioPin(GPIOC_BASE, 1);

constexpr ADS1118::ADS1118Mux channel1 = ADS1118::ADS1118Mux::MUX_AIN2_GND;
constexpr ADS1118::ADS1118Mux channel2 = ADS1118::ADS1118Mux::MUX_AIN3_GND;

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
    SPIBusConfig spiConfig = ADS1118::getDefaultSPIConfig();
    SPIBus spiBus(SPI2);
    SPISlave spiSlave(spiBus, csPin, spiConfig);

    // Device initialization
    ADS1118 ads1118(spiSlave);

    // Initialize the device
    ads1118.init();

    // Enable the two channels and the temperature
    ads1118.enableInput(channel1, ADS1118::ADS1118DataRate::DR_250,
                        ADS1118::ADS1118Pga::FSR_4_096);
    ads1118.enableInput(channel2, ADS1118::ADS1118DataRate::DR_250,
                        ADS1118::ADS1118Pga::FSR_4_096);
    ads1118.enableTemperature();

    // Self test
    if (ads1118.selfTest())
    {
        TRACE("Self test successful!\n");
    }
    else
    {
        TRACE("Self test failed :( error: %d\n", ads1118.getLastError());
    }

    // Read the two channels indipendently
    TRACE("\nNow reading the two channels with 'readInputAndWait()'\n");
    TRACE("Channel 1: %f\n", ads1118.readInputAndWait(channel1).voltage);
    TRACE("Channel 2: %f\n", ads1118.readInputAndWait(channel2).voltage);

    // Read samples with sample()
    TRACE("\nNow reading the two channels with 'sample()'\n");
    for (auto i = 0; i < 500; i++)
    {
        // Call 3 times sample() to read the 3 enabled channels
        ads1118.sample();
        miosix::Thread::sleep(5);
        ads1118.sample();
        miosix::Thread::sleep(5);
        ads1118.sample();

        printf("%.2f\t%.2f\t%.2f\n", ads1118.getTemperature().temp,
               ads1118.getVoltage(channel1).voltage,
               ads1118.getVoltage(channel2).voltage);
    }

    // Read a few times the first channel and then switch to the second one
    TRACE(
        "\nNow reading the first channel and then the second one with "
        "'sample()'\n");

    ads1118.disableAllInputs();
    ads1118.enableInput(channel1, ADS1118::ADS1118DataRate::DR_250,
                        ADS1118::ADS1118Pga::FSR_4_096);

    TRACE("First channel:\n");
    for (auto i = 0; i < 500; i++)
    {
        ads1118.sample();
        miosix::Thread::sleep(5);

        printf("%.2f\n", ads1118.getVoltage(channel1).voltage);
    }

    ads1118.disableAllInputs();
    ads1118.enableInput(channel2, ADS1118::ADS1118DataRate::DR_250,
                        ADS1118::ADS1118Pga::FSR_4_096);

    TRACE("Second channel:\n");
    for (auto i = 0; i < 500; i++)
    {
        ads1118.sample();
        miosix::Thread::sleep(5);

        printf("%.2f\n", ads1118.getVoltage(channel2).voltage);
    }
}