/* Copyright (c) 2020 Skyward Experimental Rocketry
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

#include <drivers/spi/SPI.h>
#include <miosix.h>

using namespace miosix;
using namespace Boardcore;

GpioPin sckPin  = GpioPin(GPIOE_BASE, 4);
GpioPin misoPin = GpioPin(GPIOE_BASE, 2);
GpioPin mosiPin = GpioPin(GPIOE_BASE, 5);
GpioPin csPin   = GpioPin(GPIOE_BASE, 6);

uint16_t counter = 0;

int main()
{
    // Enable clock for SPI4 interface
    RCC->APB2ENR |= RCC_APB2ENR_SPI4EN;

    // Setup gpio pins
    csPin.mode(Mode::ALTERNATE);
    csPin.alternateFunction(5);
    sckPin.mode(Mode::ALTERNATE);
    sckPin.alternateFunction(5);
    misoPin.mode(Mode::ALTERNATE);
    misoPin.alternateFunction(5);
    mosiPin.mode(Mode::ALTERNATE);
    mosiPin.alternateFunction(5);

    // Setup spi as a slave
    SPI spi(SPI4);
    spi.reset();
    spi.enable();
    spi.set16BitFrameFormat();

    printf("Slave started\n");

    while (true)
    {
        spi.getSpi()->DR = counter;
        counter++;
        // Wait until Tx buffer is empty and until the peripheral is still busy
        while ((spi.getSpi()->SR & SPI_SR_TXE) == 0)
            ;
    }
}