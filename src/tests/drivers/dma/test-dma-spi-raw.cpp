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

#include <miosix.h>

using namespace miosix;

static bool dma_complete = false;
static bool dma_error    = false;

static uint8_t txData[]  = {0xA8, 0x00};
static uint8_t rxData[2] = {0};

GpioPin csPin   = GpioPin(GPIOC_BASE, 1);
GpioPin sckPin  = GpioPin(GPIOF_BASE, 7);
GpioPin misoPin = GpioPin(GPIOF_BASE, 8);
GpioPin mosiPin = GpioPin(GPIOF_BASE, 9);

int main()
{
    // SPI INIT
    {
        csPin.mode(Mode::OUTPUT);
        csPin.high();
        sckPin.mode(Mode::ALTERNATE);
        sckPin.alternateFunction(5);
        mosiPin.mode(Mode::ALTERNATE);
        mosiPin.alternateFunction(5);
        misoPin.mode(Mode::ALTERNATE);
        misoPin.alternateFunction(5);

        // Enable SPI clock for SPI5 interface
        RCC->APB2ENR |= RCC_APB2ENR_SPI5EN;

        // Clear configuration for SPI interface
        SPI5->CR1 = 0;
        SPI5->CR2 = 0;

        // 1: Set BR[2:0] bits to define the baud rate
        SPI5->CR1 |= SPI_CR1_BR;

        // 2: Setup clock phase and polarity (MODE1: CPOL = 1, CPHA = 1)
        SPI5->CR1 |= SPI_CR1_CPOL | SPI_CR1_CPHA;

        // 7: Set MSTR to set master mode
        SPI5->CR1 |= SPI_CR1_SSI | SPI_CR1_SSM | SPI_CR1_MSTR;

        // 8: Enable DMA for tx
        SPI5->CR2 |= SPI_CR2_TXDMAEN;
        SPI5->CR2 |= SPI_CR2_RXDMAEN;

        // Enable SPI
        SPI5->CR1 |= SPI_CR1_SPE;

        printf("SPI interface setup completed...\n");
    }

    // 0: Enable DMA2 clock
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    // DMA2 Stream4 channel 3 for SPI5 TX
    {
        // 1: Disable the stream

        // Disable stream
        DMA2_Stream4->CR &= ~DMA_SxCR_EN;

        // Wait for the stream to be disabled
        while (DMA2_Stream4->CR & DMA_SxCR_EN)
            ;

        // Reset DMA configuration
        DMA2_Stream4->CR = 0;

        // Ensures all the status bits are cleared by explicitly clearing them
        DMA2->LIFCR = 0x0F7D0F7D;  // Mask excluding reserved bits
        DMA2->HIFCR = 0x0F7D0F7D;

        // 2: Set the peripheral address
        DMA2_Stream4->PAR = (uint32_t) & (SPI5->DR);

        // 3: Set the memory address
        DMA2_Stream4->M0AR = (uint32_t)txData;

        // 4: Configure the total number of data items
        DMA2_Stream4->NDTR = 2;

        // 5: Select the DMA channel
        DMA2_Stream4->CR |= DMA_SxCR_CHSEL_1;  // Channel 2 for SPI5 Tx

        // 7: Configure the stream priority to very high
        DMA2_Stream4->CR |= DMA_SxCR_PL;

        // 9: Other configuration

        // Data transfer memory-to-peripheral
        DMA2_Stream4->CR |= DMA_SxCR_DIR_0;

        // Address increment mode
        DMA2_Stream4->CR |= DMA_SxCR_MINC;

        // Interrupts (transfer complete)
        DMA2_Stream4->CR |= DMA_SxCR_TCIE;

        // Register interrupt
        NVIC_EnableIRQ(DMA2_Stream4_IRQn);
        NVIC_SetPriority(DMA2_Stream4_IRQn, 15);
    }

    // DMA2 Stream3 channel 3 for SPI5 RX
    {
        // 1: Disable the stream

        // Disable stream
        DMA2_Stream3->CR &= ~DMA_SxCR_EN;

        // Wait for the stream to be disabled
        while (DMA2_Stream3->CR & DMA_SxCR_EN)
            ;

        // Reset DMA configuration
        DMA2_Stream3->CR = 0;

        // Ensures all the status bits are cleared by explicitly clearing them
        DMA1->LIFCR = 0x0F7D0F7D;  // Mask excluding reserved bits
        DMA1->HIFCR = 0x0F7D0F7D;

        // 2: Set the peripheral address
        DMA2_Stream3->PAR = (uint32_t) & (SPI5->DR);

        // 3: Set the memory address
        DMA2_Stream3->M0AR = (uint32_t)rxData;

        // 4: Configure the total number of data items
        DMA2_Stream3->NDTR = 2;

        // 5: Select the DMA channel
        DMA2_Stream3->CR |= DMA_SxCR_CHSEL_1;  // Channel 2 for SPI5 Rx

        // 7: Configure the stream priority to very high
        DMA2_Stream3->CR |= DMA_SxCR_PL;

        // 9: Other configuration

        // Address increment mode
        DMA2_Stream3->CR |= DMA_SxCR_MINC;
    }

    // Transaction 1
    while (true)
    {
        DMA2->LIFCR = 0x0F7D0F7D;
        DMA2->HIFCR = 0x0F7D0F7D;

        csPin.low();

        // Enable DMA to start serving requests from SPI interface
        DMA2_Stream3->CR |= DMA_SxCR_EN;
        DMA2_Stream4->CR |= DMA_SxCR_EN;

        delayUs(1);

        // Wait for completion
        while ((SPI5->SR & SPI_SR_TXE) == 0)
            ;
        while (SPI5->SR & SPI_SR_BSY)
            ;

        csPin.high();

        if (dma_error)
        {
            dma_error = false;
            printf("DMA transfer error!\n");
        }

        if (dma_complete)
        {
            dma_error = false;
            printf("DMA transfer completed!\n");
        }

        printf("Rx data: 0x%02X%02X\n", rxData[0], rxData[1]);
        rxData[0] = 0;
        rxData[1] = 0;

        delayMs(1000);
    }
}

void __attribute__((naked)) DMA2_Stream4_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z27DMA2_Stream4_IRQHandlerImplv");
    restoreContext();
}

void __attribute__((used)) DMA2_Stream4_IRQHandlerImpl()
{
    if (DMA2->HISR & DMA_HISR_TCIF4)
    {
        dma_complete = true;

        // Clear the interrupt
        SET_BIT(DMA2->HIFCR, DMA_HIFCR_CTCIF4);
    }

    if (DMA2->HISR & DMA_HISR_TEIF4)
    {
        dma_error = true;

        // Clear the interrupt
        SET_BIT(DMA2->HIFCR, DMA_HIFCR_CTEIF4);
    }
}
