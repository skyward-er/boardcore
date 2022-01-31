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

#include <drivers/dma/DMAStream.h>
#include <drivers/spi/SPI.h>
#include <miosix.h>

/**
 * This test reads the gyroscope on the Discovery 429 using SPI with DMA.
 */

using namespace miosix;
using namespace Boardcore;

GpioPin csPin   = GpioPin(GPIOC_BASE, 1);
GpioPin sckPin  = GpioPin(GPIOF_BASE, 7);
GpioPin misoPin = GpioPin(GPIOF_BASE, 8);
GpioPin mosiPin = GpioPin(GPIOF_BASE, 9);

SPI spi5 = SPI(SPI5);

DMAStream txStream = DMAStream(DMA2_Stream4);
DMAStream rxStream = DMAStream(DMA2_Stream3);

static uint8_t txData[]  = {0xA8, 0x00};
static uint8_t rxData[2] = {0};

void initGPIOs();
void initSPI();
void initDMA();

int main()
{
    initGPIOs();
    initSPI();
    initDMA();

    while (true)
    {
        csPin.low();

        // Enable DMA to start serving requests from SPI interface
        rxStream.enable();
        txStream.enable();

        delayUs(1);

        // Wait for completion
        spi5.waitPeripheral();

        csPin.high();

        printf("Rx data: 0x%02X%02X\n", rxData[0], rxData[1]);
        rxData[0] = 0;
        rxData[1] = 0;

        delayMs(1000);
    }
}

void initGPIOs()
{
    csPin.mode(Mode::OUTPUT);
    csPin.high();
    sckPin.mode(Mode::ALTERNATE);
    sckPin.alternateFunction(5);
    mosiPin.mode(Mode::ALTERNATE);
    mosiPin.alternateFunction(5);
    misoPin.mode(Mode::ALTERNATE);
    misoPin.alternateFunction(5);
}

void initSPI()
{
    spi5.reset();
    spi5.setClockDiver(SPI::ClockDivider::DIV_256);
    spi5.setMode(SPI::Mode::MODE_3);
    spi5.enableInternalSlaveSelection();
    spi5.enableSoftwareSlaveManagement();
    spi5.setMasterConfiguration();
    spi5.enableTxDMARequest();
    spi5.enableRxDMARequest();
    spi5.enable();
}

void initDMA()
{
    // 0: Enable DMA2 clock
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    // DMA2 Stream4 channel 2 for SPI5 Tx
    {
        txStream.reset();
        txStream.setPeripheralAddress(const_cast<uint32_t*>(&(SPI5->DR)));
        txStream.setMemory0Address(reinterpret_cast<uint32_t*>(txData));
        txStream.setNumberOfDataItems(2);
        txStream.setStreamChannel(DMAStream::Channel::CHANNEL2);
        txStream.setStreamPriorityLevel(DMAStream::PriorityLevel::VERY_HIGH);
        txStream.setDataTransferDirection(
            DMAStream::DataTransferDirection::MEM_TO_PERIPH);
        txStream.enableMemoryIncrement();
    }

    // DMA2 Stream3 channel 2 for SPI5 Rx
    {
        rxStream.reset();
        rxStream.setPeripheralAddress(const_cast<uint32_t*>(&(SPI5->DR)));
        rxStream.setMemory0Address(reinterpret_cast<uint32_t*>(rxData));
        rxStream.setNumberOfDataItems(2);
        rxStream.setStreamChannel(DMAStream::Channel::CHANNEL2);
        rxStream.setStreamPriorityLevel(DMAStream::PriorityLevel::VERY_HIGH);
        rxStream.enableMemoryIncrement();
    }
}
