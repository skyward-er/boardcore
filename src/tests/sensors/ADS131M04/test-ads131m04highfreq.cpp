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
using namespace Boardcore;

GpioPin sckPin  = GpioPin(GPIOE_BASE, 2);
GpioPin misoPin = GpioPin(GPIOE_BASE, 5);
GpioPin mosiPin = GpioPin(GPIOE_BASE, 6);
GpioPin csPin   = GpioPin(GPIOE_BASE, 4);

GpioPin timerCsPin  = GpioPin(GPIOA_BASE, 11);
GpioPin timerSckPin = GpioPin(GPIOB_BASE, 1);

static volatile bool dma_complete = false;

static constexpr int BUFF_SIZE = 64;

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

    SPISignalGenerator spiSignalGenerator(
        16, BUFF_SIZE, 1000000, SPI::Mode::MODE_0,
        GeneralPurposeTimer<uint16_t>::Channel::CHANNEL_1,
        GeneralPurposeTimer<uint16_t>::Channel::CHANNEL_4,
        GeneralPurposeTimer<uint16_t>::Channel::CHANNEL_4);
    SPISlaveBus spiBus(SPI4, spiSignalGenerator);
    SPISlave spiSlave(spiBus, csPin, {});
    ADS131M04HighFreq ads131(spiSlave, SPI4, (DMA2_Stream0),
                             DMAStream::Channel::CHANNEL4, spiSignalGenerator,
                             BUFF_SIZE);

    // Setup transfer complete interrupt
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    NVIC_SetPriority(DMA2_Stream0_IRQn, 15);

    ads131.startHighFreqSampling();

    while (true)
    {
        delayUs(1);
        while (!dma_complete)
            ;

        dma_complete = false;

        ads131.stopHighFreqSampling();

        printf("Transfer complete! %ld\n", DMA2_Stream0->CR & DMA_SxCR_CT);

        // Print the entire buffer
        printf("\tBuffer 1:\n");
        for (int i = 0; i < BUFF_SIZE; i++)
        {
            printf("%4X %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X %4X\n",
                   ads131.buffer1[i].status, ads131.buffer1[i].rawData[0],
                   ads131.buffer1[i].rawData[1], ads131.buffer1[i].rawData[2],
                   ads131.buffer1[i].rawData[3], ads131.buffer1[i].rawData[4],
                   ads131.buffer1[i].rawData[5], ads131.buffer1[i].rawData[6],
                   ads131.buffer1[i].rawData[7], ads131.buffer1[i].rawData[8],
                   ads131.buffer1[i].rawData[9], ads131.buffer1[i].rawData[10],
                   ads131.buffer1[i].rawData[11], ads131.buffer1[i].crc);
        }
        printf("\tBuffer 2:\n");
        for (int i = 0; i < BUFF_SIZE; i++)
        {
            printf("%4X %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X %4X\n",
                   ads131.buffer2[i].status, ads131.buffer2[i].rawData[0],
                   ads131.buffer2[i].rawData[1], ads131.buffer2[i].rawData[2],
                   ads131.buffer2[i].rawData[3], ads131.buffer2[i].rawData[4],
                   ads131.buffer2[i].rawData[5], ads131.buffer2[i].rawData[6],
                   ads131.buffer2[i].rawData[7], ads131.buffer2[i].rawData[8],
                   ads131.buffer2[i].rawData[9], ads131.buffer2[i].rawData[10],
                   ads131.buffer2[i].rawData[11], ads131.buffer2[i].crc);
        }

        ads131.resumeHighFreqSampling();
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
