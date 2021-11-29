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

#pragma once

#include <drivers/dma/DMAStream.h>
#include <drivers/spi/SPISignalGenerator.h>

#include "ADS131M04.h"
#include "ADS131M04HighFreqData.h"

namespace Boardcore
{

/**
 * @brief Driver for ADS131M04 in combination with DMA and timers.
 */
class ADS131M04HighFreq : public ADS131M04
{
public:
    uint16_t buffer1[12] = {0};
    uint16_t buffer2[12] = {0};

    ADS131M04HighFreq(SPISlave spiSlave, SPIType *spi, DMAStream rxStream,
                      DMAStream::Channel dmaChannel);

    void startHighFreqSampling();

private:
    SPI spi;
    DMAStream rxStream;
    DMAStream::Channel dmaChannel;
    bool highFreqSamplingStarted = false;
};

}  // namespace Boardcore
