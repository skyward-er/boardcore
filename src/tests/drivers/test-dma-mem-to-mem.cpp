/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Authors: Alberto Nidasio, Fabrizio Monti
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

using namespace miosix;
using namespace Boardcore;

void printBuffer(uint8_t* buffer, size_t size);

int main()
{
    printf("ok funziona sulla engine\n");
    // SCB_DisableDCache();
    // SCB_DisableICache();
    // miosix::Thread::sleep(500);

    /**
     * verificare quali stream sono usate sull f7 e quali no (provando
     * a compilare avendo tutti gli irq handler attivi)
     *
     * è giusto che non runni i callback in polling mode?
     */

    // DMAStreamGuard stream = DMADriver::instance().acquireStreamForPeripheral(
    //     DMADefs::Peripherals::PE_MEM_ONLY);
    DMAStreamGuard stream = DMADriver::instance().acquireStream(DMADefs::DMAStreamId::DMA2_Str0, DMADefs::Channel::CHANNEL0);

    if (!stream.isValid())
    {
        printf("Error, cannot allocate dma stream\n");
        return 0;
    }

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
    stream->setup(trn);

    stream->enable();

    // auto start = std::chrono::steady_clock::now();
    stream->waitForTransferComplete();
    // auto end = std::chrono::steady_clock::now();

    // auto dur = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    // printf("tempo impiegato: %lld ns\n\n", dur.count());

    // stream->readFlags();
    // printf("direct mode error flag: %d\n", stream->getDirectModeErrorFlagStatus());
    // printf("fifo error flag: %d\n", stream->getFifoErrorFlagStatus());
    // printf("half trans flag: %d\n", stream->getHalfTransferFlagStatus());
    // printf("trans complete flag: %d\n", stream->getTransferCompleteFlagStatus());
    // printf("trans error flag: %d\n\n", stream->getTransferErrorFlagStatus());

    SCB_InvalidateDCache_by_Addr((uint32_t*)buffer2, 8); // risolve solo per i primi 4 byte
    // SCB_InvalidateDCache_by_Addr((uint32_t*)(buffer2+4), 8); // risolve solo per gli ultimi 4 byte
    // SCB_CleanDCache_by_Addr((uint32_t*)buffer2, 8); // non fa nulla

    if ( ((uint64_t)buffer2 & 0x3) == 0 )
    {
        // The address is 4-byte aligned here
        printf("buf2 is 32-bit aligned\n");
    }
    else
    {
        printf("buf2 IS NOT 32-bit aligned\n");
    }


    printf("After:\n");
    printf("Buffer 1:\n");
    printBuffer(buffer1, sizeof(buffer1));
    printf("Buffer 2:\n");
    printBuffer(buffer2, sizeof(buffer2));

    return 0;
}

void printBuffer(uint8_t* buffer, size_t size)
{
    for (size_t i = 0; i < size - 1; i++)
        printf("%x,", buffer[i]);

    printf("%x\n", buffer[size - 1]);
}
