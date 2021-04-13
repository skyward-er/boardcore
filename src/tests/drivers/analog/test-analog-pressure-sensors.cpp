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

#include <miosix.h>

#include "Common.h"
#include "drivers/adc/ADS1118/ADS1118.h"
#include "drivers/spi/SPIDriver.h"
#include "sensors/analog/pressure/AnalogPressureSensor.h"
#include "sensors/analog/pressure/honeywell/HSCMAND015PA.h"
#include "sensors/analog/pressure/honeywell/HSCMRNN030PA.h"

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
    SPIBusConfig spiConfig;
    spiConfig.clock_div = SPIClockDivider::DIV32;
    spiConfig.mode      = SPIMode::MODE1;
    SPIBus spiBus(SPI2);
    SPISlave spiSlave(spiBus, csPin, spiConfig);

    // Device initialization
    ADS1118 ads1118(spiSlave);

    std::function<ADCData()> get_voltage_function =
        std::bind(&ADS1118::getVoltage, ads1118, channel1);
    HSCMAND015PA analog_sensor(get_voltage_function);

    analog_sensor.init();
    analog_sensor.selfTest();

    std::function<ADCData()> get_voltage_function2 =
        std::bind(&ADS1118::getVoltage, ads1118, channel2);
    HSCMRNN030PA analog_sensor2(get_voltage_function2);

    analog_sensor2.init();
    analog_sensor2.selfTest();

    // Enable channels
    ads1118.enableInput(channel1, ADS1118::ADS1118DataRate::DR_250,
                        ADS1118::ADS1118Pga::FSR_4_096);
    ads1118.enableInput(channel2, ADS1118::ADS1118DataRate::DR_250,
                        ADS1118::ADS1118Pga::FSR_4_096);

    // Initialize the device
    ads1118.init();

    // Self test
    if (ads1118.selfTest())
    {
        TRACE("Self test successful!\n");
    }
    else
    {
        TRACE("Self test failed :( error: %d\n", ads1118.getLastError());
    }

    // Read samples with sample()
    while (1)
    {
        ads1118.sample();
        miosix::Thread::sleep(5);

        printf("%.2f\t%.2f\n", ads1118.getVoltage(channel1).voltage,
               ads1118.getVoltage(channel2).voltage);
    }
}