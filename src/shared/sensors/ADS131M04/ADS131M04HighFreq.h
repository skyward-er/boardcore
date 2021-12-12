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

#include <ActiveObject.h>
#include <diagnostic/PrintLogger.h>
#include <drivers/dma/DMAStream.h>
#include <drivers/spi/SPISignalGenerator.h>

#include "ADS131M04.h"
#include "ADS131M04HighFreqData.h"

namespace Boardcore
{

/**
 * @brief Driver for ADS131M04 in combination with DMA and timers.
 */
class ADS131M04HighFreq : public ADS131M04, private ActiveObject
{
public:
    ADS131M04HighFreqData* buffer1 = nullptr;
    ADS131M04HighFreqData* buffer2 = nullptr;

    ADS131M04HighFreq(SPISlave spiSlave, SPIType* spi, DMAStream rxStream,
                      DMAStream::Channel dmaChannel,
                      SPISignalGenerator spiSignalGenerator, int bufSize,
                      std::string logFileName);

    /**
     * @brief Configures DMA and SPI peripherals and starts sampling by enablig
     * the SPI signal generator.
     */
    void startHighFreqSampling();

    /**
     * @brief Stops the high frequency sampling by disabling the SPI signal
     * generator.
     *
     * After calling this function DMA and SPI are still correctly configured
     * and transaction can be restarted with resumeHighFreqSampling.
     */
    void stopHighFreqSampling();

    /**
     * @brief Resumed the high frequency sampling by enabling the SPI signal
     * generator.
     *
     * DMA and SPI must be already configured, otherwise no data will be read.
     */
    void resumeHighFreqSampling();

    void startLogging();

    void waitDataAndThenLog(void*);

    void handleTransferCompleteInterrupt();

protected:
    void run() override;

private:
    SPI spi;
    DMAStream rxStream;
    DMAStream::Channel dmaChannel;
    SPISignalGenerator spiSignalGenerator;
    int bufSize;
    std::string logFileName;

    bool highFreqSamplingStarted = false;

    miosix::Thread* loggerThread = nullptr;

    PrintLogger logger = Logging::getLogger("ads131m04");
};

}  // namespace Boardcore
