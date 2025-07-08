/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Pietro Bortolus
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

#include <drivers/dma/DMA.h>
#include <miosix.h>

#include <iostream>

#include "sensors/ND015X/ND015A.h"
#include "sensors/ND015X/ND015D.h"

using namespace miosix;
using namespace Boardcore;

// Pin definitions for Lyra Biscotto
GpioPin sckPin      = GpioPin(GPIOE_BASE, 2);
GpioPin misoPin     = GpioPin(GPIOE_BASE, 5);
GpioPin mosiPin     = GpioPin(GPIOE_BASE, 6);
GpioPin csPinND015A = GpioPin(GPIOB_BASE, 9);
GpioPin csPinND015D = GpioPin(GPIOB_BASE, 8);

void initPins()
{
    // Setup gpio pins

    sckPin.alternateFunction(5);
    sckPin.mode(Mode::ALTERNATE);
    misoPin.alternateFunction(5);
    misoPin.mode(Mode::ALTERNATE);
    mosiPin.alternateFunction(5);
    mosiPin.mode(Mode::ALTERNATE);
    csPinND015A.mode(Mode::OUTPUT);
    csPinND015A.high();
    csPinND015D.mode(Mode::OUTPUT);
    csPinND015D.high();
}

int main()
{
    // Initialize SPI pins
    initPins();

    SPIBus bus(SPI4);

    auto streamRx = DMADriver::instance().acquireStreamForPeripheral(
        DMADefs::Peripherals::PE_SPI4_RX);
    if (!streamRx.isValid())
    {
        std::cout << "Cannot acquire streamRx" << std::endl;
        return 1;
    }

    auto streamTx = DMADriver::instance().acquireStreamForPeripheral(
        DMADefs::Peripherals::PE_SPI4_TX);
    if (!streamTx.isValid())
    {
        std::cout << "Cannot acquire streamTx" << std::endl;
        return 1;
    }

    ND015A sensor(bus, csPinND015A, ND015A::getDefaultSPIConfig(), &streamRx,
                  &streamTx, bus.getSpi(), std::chrono::milliseconds(100));
    // ND015D sensor(bus, csPinND015D, ND015D::getDefaultSPIConfig(), &streamRx,
    //                 &streamTx, bus.getSpi(), std::chrono::milliseconds(100));
    // ND015A sensor(bus, csPinND015A, ND015A::getDefaultSPIConfig());
    // ND015D sensor(bus, csPinND015D, ND015D::getDefaultSPIConfig());
    ND015XData sensorData;

    sensor.init();

    if (sensor.checkModelMatch())
    {
        std::cout << "Sensor initialized correctly" << std::endl;
    }
    else
    {
        std::cout << "Sensor failed to initialize" << std::endl;
        return 1;
    }

    while (true)
    {
        sensor.sample();
        sensorData = sensor.getLastSample();
        std::cout << "New data: " << sensorData.pressure << std::endl;

        Thread::sleep(1000);
    }

    return 0;
}
