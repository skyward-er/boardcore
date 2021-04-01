/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Luca Conterio
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

#include <Common.h>
#include <cstdio>
#include <iostream>
#include <sstream>
#include "drivers/spi/SPIDriver.h"

using namespace std;
using namespace miosix;

SPIBus bus(SPI1);

GpioPin spi_sck(GPIOA_BASE, 5);
GpioPin spi_miso(GPIOA_BASE, 6);
GpioPin spi_mosi(GPIOA_BASE, 7);
GpioPin cs(GPIOE_BASE, 3);

int main()
{
    {
        miosix::FastInterruptDisableLock dLock;

        RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;  // Enable SPI1 bus

        spi_sck.mode(miosix::Mode::ALTERNATE);
        spi_sck.alternateFunction(5);
        spi_miso.mode(miosix::Mode::ALTERNATE);
        spi_miso.alternateFunction(5);
        spi_mosi.mode(miosix::Mode::ALTERNATE);
        spi_mosi.alternateFunction(5);

        cs.mode(miosix::Mode::OUTPUT);
    }
    cs.high();

    SPISlave spi_slave(bus, cs, {});
    spi_slave.config.clock_div = SPIClockDivider::DIV64;

    while (true)
    {
        printf("Insert a number (0-255): \n");

        string temp;
        int cmd;
        getline(cin, temp);
        stringstream(temp) >> cmd;

        {
            SPITransaction spi(spi_slave);

            // spam over spi in order to easily find
            // the data with a logic analyzer
            for (int i = 0; i < 10; i++)
            {
                for (int i = 0; i < 10; i++)
                {
                    spi.write((uint8_t)cmd);
                }

                Thread::sleep(10);
            }
        }

        Thread::sleep(100);
    }

    return 0;
}
