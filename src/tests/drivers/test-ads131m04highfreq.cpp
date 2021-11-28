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

#include <drivers/dma/DMAStream.h>
#include <sensors/ADS131M04/ADS131M04HighFreq.h>

using namespace miosix;

GpioPin sckPin  = GpioPin(GPIOE_BASE, 4);
GpioPin misoPin = GpioPin(GPIOE_BASE, 2);
GpioPin mosiPin = GpioPin(GPIOE_BASE, 5);
GpioPin csPin   = GpioPin(GPIOE_BASE, 6);

GpioPin timerCsPin  = GpioPin(GPIOA_BASE, 11);
GpioPin timerSckPin = GpioPin(GPIOB_BASE, 1);

static volatile bool dma_complete = false;

int main()
{
    // Enable clock for SPI4 interface
    RCC->APB2ENR |= RCC_APB2ENR_SPI4EN;

    // Enable clock for DMA1
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    // Enable clock for timers TIM1 and TIM3
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // Setup gpio pins
    csPin.mode(Mode::ALTERNATE);
    csPin.alternateFunction(5);
    sckPin.mode(Mode::ALTERNATE);
    sckPin.alternateFunction(5);
    misoPin.mode(Mode::ALTERNATE);
    misoPin.alternateFunction(5);
    mosiPin.mode(Mode::ALTERNATE);
    mosiPin.alternateFunction(5);

    timerCsPin.mode(Mode::ALTERNATE);
    timerCsPin.alternateFunction(1);
    timerSckPin.mode(Mode::ALTERNATE);
    timerSckPin.alternateFunction(2);

    SPISignalGenerator<1, 4, 4> spiSignalGenerator{2, 100, 1000000};
    SPIBus spiBus(SPI4);
    SPISlave spiSlave(spiBus, csPin, {});
    DMAStream rxStream(DMA2_Stream0);
    ADS131M04HighFreq ads131(spiSlave, SPI4, rxStream,
                             DMAStream::Channel::CHANNEL4);

    // Setup transfer complete interrupt
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    NVIC_SetPriority(DMA2_Stream0_IRQn, 15);

    ads131.startHighFreqSampling();

    // Start the clock generator
    spiSignalGenerator.configure();
    spiSignalGenerator.enable();

    while (true)
    {
        delayUs(1);
        while (!dma_complete)
            ;

        dma_complete = false;

        printf("Transfer complete!\n");
        printf("Buffer 1: %2X,%2X,%2X,%2X,%2X,%2X,%2X,%2X,%2X,%2X,%2X,%2X\n",
               ads131.buffer1[0], ads131.buffer1[1], ads131.buffer1[2],
               ads131.buffer1[3], ads131.buffer1[4], ads131.buffer1[5],
               ads131.buffer1[6], ads131.buffer1[7], ads131.buffer1[8],
               ads131.buffer1[9], ads131.buffer1[10], ads131.buffer1[11]);
        printf("Buffer 2: %2X,%2X,%2X,%2X,%2X,%2X,%2X,%2X,%2X,%2X,%2X,%2X\n",
               ads131.buffer2[0], ads131.buffer2[1], ads131.buffer2[2],
               ads131.buffer2[3], ads131.buffer2[4], ads131.buffer2[5],
               ads131.buffer2[6], ads131.buffer2[7], ads131.buffer2[8],
               ads131.buffer2[9], ads131.buffer2[10], ads131.buffer2[11]);
    }
}

void __attribute__((naked)) DMA2_Stream0_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z27DMA2_Stream0_IRQHandlerImplv");
    restoreContext();
}

void __attribute__((used)) DMA2_Stream0_IRQHandlerImpl()
{
    dma_complete = true;
    if (DMA2->LISR & DMA_LISR_TCIF0)
    {

        // Clear the interrupt
        SET_BIT(DMA2->LIFCR, DMA_LIFCR_CTCIF0);
    }
}
