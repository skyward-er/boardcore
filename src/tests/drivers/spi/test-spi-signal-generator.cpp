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

#include <drivers/spi/SPISignalGenerator.h>
#include <miosix.h>

using namespace miosix;

using namespace TimerUtils;

/**
 * Pin list:
 *   PB4 - TIM3_CH1 - CS signal
 *   PE5 - TIM9_CH1 - SCK signal
 *
 * We'll use two timers to generate clock and chip select signals for the SPI
 * peripheral.
 */

GpioPin csPin  = GpioPin(GPIOB_BASE, 14);
GpioPin sckPin = GpioPin(GPIOB_BASE, 1);

void setupGPIOs();

int main()
{
    // Enable timers clock
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    }

    setupGPIOs();

    SPISignalGenerator<> spiSignalGenerator{2, 1000};
    spiSignalGenerator.configure();
    spiSignalGenerator.enable();

    Thread::sleep(5 * 1000);

    spiSignalGenerator.disable();

    while (true)
        Thread::sleep(10000);
}

void setupGPIOs()
{
    csPin.mode(Mode::ALTERNATE);
    csPin.alternateFunction(1);

    sckPin.mode(Mode::ALTERNATE);
    sckPin.alternateFunction(2);
}
