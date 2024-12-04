/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Authors: Alberto Nidasio
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

#include <drivers/dma/DMA.h>
#include <miosix.h>
#include <util/util.h>

#include <chrono>

using namespace miosix;
using namespace Boardcore;

DMAStream &stream = DMADriver::instance().acquireStream(DMAStreamId::DMA2_Str0);

void printBuffer(uint8_t *buffer, size_t size);

int main()
{
    /**
     * In this test we want to copy a buffer1 into buffer2 with the DMA.
     */

    uint8_t buffer1[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    uint8_t buffer2[8] = {0};

    printf("Before:\n");
    printf("Buffer 1:\n");
    printBuffer(buffer1, sizeof(buffer1));
    printf("Buffer 2:\n");
    printBuffer(buffer2, sizeof(buffer2));

    DMATransaction trn{
        .direction         = DMATransaction::Direction::MEM_TO_MEM,
        .srcSize           = DMATransaction::DataSize::BITS_8,
        .dstSize           = DMATransaction::DataSize::BITS_8,
        .srcAddress        = buffer1,
        .dstAddress        = buffer2,
        .numberOfDataItems = sizeof(buffer1),
        .srcIncrement      = true,
        .dstIncrement      = true,
        .enableTransferCompleteInterrupt = true,
    };
    stream.setup(trn);
    stream.enable();

    auto begin = std::chrono::steady_clock::now();
    stream.waitForTransferComplete();
    auto end = std::chrono::steady_clock::now();

    printf("After:\n");
    printf("Buffer 1:\n");
    printBuffer(buffer1, sizeof(buffer1));
    printf("Buffer 2:\n");
    printBuffer(buffer2, sizeof(buffer2));

    const auto elapsedTime =
        std::chrono::duration_cast<std::chrono::microseconds>(end - begin)
            .count();
    printf("Elapsed time: %lld [us]\n", elapsedTime);

    return 0;
}

void printBuffer(uint8_t *buffer, size_t size)
{
    for (size_t i = 0; i < size - 1; i++)
    {
        printf("%x,", buffer[i]);
    }

    printf("%x\n", buffer[size - 1]);
}
