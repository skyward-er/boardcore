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

#include "ADS131M04HighFreq.h"

namespace Boardcore
{

ADS131M04HighFreq::ADS131M04HighFreq(SPISlave spiSlave, SPIType *spi,
                                     DMAStream rxStream,
                                     DMAStream::Channel dmaChannel)
    : ADS131M04(spiSlave), spi(spi), rxStream(rxStream), dmaChannel(dmaChannel)
{
}

void ADS131M04HighFreq::startHighFreqSampling()
{
    // Setup spi as a slave
    spi.reset();
    spi.set16BitFrameFormat();
    spi.setMode(spiSlave.config.mode);
    spi.enableRxDMARequest();
    spi.enable();

    // Setup DMA stream
    rxStream.reset();
    rxStream.setPeripheralAddress(const_cast<uint32_t *>(&(spi.getSpi()->DR)));
    rxStream.setMemory0Address(reinterpret_cast<uint32_t *>(buffer1));
    rxStream.setMemory1Address(reinterpret_cast<uint32_t *>(buffer2));
    rxStream.setMemoryDataSize(DMAStream::MemoryDataSize::HALF_WORD);
    rxStream.setPeripheralDataSize(DMAStream::PeripheralDataSize::HALF_WORD);
    rxStream.setNumberOfDataItems(12);
    rxStream.setStreamChannel(dmaChannel);
    rxStream.setStreamPriorityLevel(DMAStream::PriorityLevel::VERY_HIGH);
    rxStream.enableMemoryIncrement();
    rxStream.enableTransferCompleteInterrupt();
    rxStream.enableCircularMode();
    rxStream.enableDoubleBufferMode();
    rxStream.enable();
}

}  // namespace Boardcore
