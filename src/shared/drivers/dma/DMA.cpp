/* Copyright (c) 2023 Skyward Experimental Rocketry
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

#include "DMA.h"

#include <kernel/logging.h>
#include <utils/ClockUtils.h>

#include <map>

using namespace miosix;

void __attribute__((naked)) DMA1_Stream0_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z20DMA1_Stream0_IRQImplv");
    restoreContext();
}

void __attribute__((used)) DMA1_Stream0_IRQImpl()
{
    Boardcore::DMADriver::instance().IRQhandleInterrupt(
        Boardcore::DMAStreamId::DMA1_Str0);
}

// Commented because already used elsewhere by miosix
// void __attribute__((naked)) DMA1_Stream1_IRQHandler()
// {
//     saveContext();
//     asm volatile("bl _Z20DMA1_Stream1_IRQImplv");
//     restoreContext();
// }
// void __attribute__((used)) DMA1_Stream1_IRQImpl()
// {
//     Boardcore::DMADriver::instance().IRQhandleInterrupt(Boardcore::DMAStreamId::DMA1_Str1);
// }

void __attribute__((naked)) DMA1_Stream2_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z20DMA1_Stream2_IRQImplv");
    restoreContext();
}

void __attribute__((used)) DMA1_Stream2_IRQImpl()
{
    Boardcore::DMADriver::instance().IRQhandleInterrupt(
        Boardcore::DMAStreamId::DMA1_Str2);
}

// Commented because already used elsewhere by miosix
// void __attribute__((naked)) DMA1_Stream3_IRQHandler()
// {
//     saveContext();
//     asm volatile("bl _Z20DMA1_Stream3_IRQImplv");
//     restoreContext();
// }
// void __attribute__((used)) DMA1_Stream3_IRQImpl()
// {
//     Boardcore::DMADriver::instance().IRQhandleInterrupt(Boardcore::DMAStreamId::DMA1_Str3);
// }

void __attribute__((naked)) DMA1_Stream4_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z20DMA1_Stream4_IRQImplv");
    restoreContext();
}

void __attribute__((used)) DMA1_Stream4_IRQImpl()
{
    Boardcore::DMADriver::instance().IRQhandleInterrupt(
        Boardcore::DMAStreamId::DMA1_Str4);
}

void __attribute__((naked)) DMA1_Stream5_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z20DMA1_Stream5_IRQImplv");
    restoreContext();
}

void __attribute__((used)) DMA1_Stream5_IRQImpl()
{
    Boardcore::DMADriver::instance().IRQhandleInterrupt(
        Boardcore::DMAStreamId::DMA1_Str5);
}

void __attribute__((naked)) DMA1_Stream6_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z20DMA1_Stream6_IRQImplv");
    restoreContext();
}

void __attribute__((used)) DMA1_Stream6_IRQImpl()
{
    Boardcore::DMADriver::instance().IRQhandleInterrupt(
        Boardcore::DMAStreamId::DMA1_Str6);
}

void __attribute__((naked)) DMA1_Stream7_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z20DMA1_Stream7_IRQImplv");
    restoreContext();
}

void __attribute__((used)) DMA1_Stream7_IRQImpl()
{
    Boardcore::DMADriver::instance().IRQhandleInterrupt(
        Boardcore::DMAStreamId::DMA1_Str7);
}

void __attribute__((naked)) DMA2_Stream0_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z20DMA2_Stream0_IRQImplv");
    restoreContext();
}

void __attribute__((used)) DMA2_Stream0_IRQImpl()
{
    Boardcore::DMADriver::instance().IRQhandleInterrupt(
        Boardcore::DMAStreamId::DMA2_Str0);
}

void __attribute__((naked)) DMA2_Stream1_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z20DMA2_Stream1_IRQImplv");
    restoreContext();
}

void __attribute__((used)) DMA2_Stream1_IRQImpl()
{
    Boardcore::DMADriver::instance().IRQhandleInterrupt(
        Boardcore::DMAStreamId::DMA2_Str1);
}

void __attribute__((naked)) DMA2_Stream2_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z20DMA2_Stream2_IRQImplv");
    restoreContext();
}

void __attribute__((used)) DMA2_Stream2_IRQImpl()
{
    Boardcore::DMADriver::instance().IRQhandleInterrupt(
        Boardcore::DMAStreamId::DMA2_Str2);
}

// void __attribute__((naked)) DMA2_Stream3_IRQHandler() {
//     saveContext();
//     asm volatile("bl _Z20DMA2_Stream3_IRQImplv");
//     restoreContext();
// }

// void __attribute__((used)) DMA2_Stream3_IRQImpl() {
//     DMADriver::instance().IRQhandleInterrupt(DMAStreamId::DMA2_Str3);
// }

void __attribute__((naked)) DMA2_Stream4_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z20DMA2_Stream4_IRQImplv");
    restoreContext();
}

void __attribute__((used)) DMA2_Stream4_IRQImpl()
{
    Boardcore::DMADriver::instance().IRQhandleInterrupt(
        Boardcore::DMAStreamId::DMA2_Str4);
}

// void __attribute__((naked)) DMA2_Stream5_IRQHandler() {
//     saveContext();
//     asm volatile("bl _Z20DMA2_Stream5_IRQImplv");
//     restoreContext();
// }

// void __attribute__((used)) DMA2_Stream5_IRQImpl() {
//     DMADriver::instance().IRQhandleInterrupt(DMAStreamId::DMA2_Str5);
// }

void __attribute__((naked)) DMA2_Stream6_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z20DMA2_Stream6_IRQImplv");
    restoreContext();
}

void __attribute__((used)) DMA2_Stream6_IRQImpl()
{
    Boardcore::DMADriver::instance().IRQhandleInterrupt(
        Boardcore::DMAStreamId::DMA2_Str6);
}

// void __attribute__((naked)) DMA2_Stream7_IRQHandler() {
//     saveContext();
//     asm volatile("bl _Z20DMA2_Stream7_IRQImplv");
//     restoreContext();
// }

// void __attribute__((used)) DMA2_Stream7_IRQImpl() {
//     DMADriver::instance().IRQhandleInterrupt(DMAStreamId::DMA2_Str7);
// }

namespace Boardcore
{

void DMADriver::IRQhandleInterrupt(DMAStreamId id)
{
    auto stream = streams[id];

    stream->readFlags();
    stream->clearAllFlags();

    // Run the callbacks if neccessary
    if (stream->halfTransferCallback && stream->halfTransferFlag)
    {
        stream->halfTransferCallback();
    }
    if (stream->transferCompleteCallback && stream->transferCompleteFlag)
    {
        stream->transferCompleteCallback();
    }
    if (stream->errorCallback &&
        (stream->transferErrorFlag || stream->fifoErrorFlag ||
         stream->directModeErrorFlag))
    {
        stream->errorCallback();
    }

    // Wakeup the thread if the user is waiting
    if (stream->waitingThread)
    {
        IRQwakeupThread(stream);
    }
}

void DMADriver::IRQwakeupThread(DMAStream* stream)
{
    // Wakeup the waiting thread
    stream->waitingThread->wakeup();

    // If the waiting thread has a higher priority than the current
    // thread then reschedule
    if (stream->waitingThread->IRQgetPriority() >
        miosix::Thread::IRQgetCurrentThread()->IRQgetPriority())
    {
        miosix::Scheduler::IRQfindNextThread();
    }

    // Clear the thread pointer, this way the thread will be sure it is
    // not a spurious wakeup
    stream->waitingThread = nullptr;
}

DMADriver& DMADriver::instance()
{
    static DMADriver instance;
    return instance;
}

bool DMADriver::tryChannel(DMAStreamId id)
{
    Lock<FastMutex> l(mutex);

    // Return true, meaning that the channel is free, only if it is not yet
    // allocated
    return streams.count(id) == 0;
}

DMAStream& DMADriver::acquireStream(DMAStreamId id)
{
    Lock<FastMutex> l(mutex);

    // Wait until the channel is free
    while (streams.count(id) != 0)
        cv.wait(l);

    // Enable the clock if not already done
    // TODO: Enable DMA1 or DMA2
    // if (streams.size() == 0)
    //     RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    return *(streams[id] = new DMAStream(id));
}

void DMADriver::releaseStream(DMAStreamId id)
{
    Lock<FastMutex> l(mutex);

    if (streams.count(id) != 0)
    {
        delete streams[id];
        streams.erase(id);
        cv.broadcast();
    }

    // Disable the clock if there are no more channels
    // TODO: Disable DMA1 or DMA2
    // if (streams.size() == 0)
    //     RCC->AHB1ENR &= ~RCC_AHB1ENR_DMA1EN;
}

DMADriver::DMADriver()
{
    // For now enable the clock always
    ClockUtils::enablePeripheralClock(DMA1);
    ClockUtils::enablePeripheralClock(DMA2);

    // Reset interrupts flags by setting the clear bits to 1
    // TODO: Change this magic number
    DMA1->HIFCR = 0x0f7d0f7d;  // = 0b 0000 1111 0111 1101 0000 1111 0111 1101
    DMA1->LIFCR = 0x0f7d0f7d;
    DMA2->HIFCR = 0x0f7d0f7d;
    DMA2->LIFCR = 0x0f7d0f7d;
}

void DMAStream::setup(DMATransaction transaction)
{
    currentSetup = transaction;

    // Reset the configuration
    registers->CR = 0;

    // Wait for the stream to actually be disabled
    while (registers->CR & DMA_SxCR_EN)
        ;

    registers->CR |= static_cast<uint32_t>(transaction.channel);
    registers->CR |= static_cast<uint32_t>(transaction.direction);
    registers->CR |= static_cast<uint32_t>(transaction.priority);
    if (transaction.circularMode)
        registers->CR |= DMA_SxCR_CIRC;
    registers->NDTR = transaction.numberOfDataItems;

    if (transaction.direction == DMATransaction::Direction::MEM_TO_PER)
    {
        // In memory to peripheral mode, the source address is the memory
        // address

        registers->CR |= static_cast<uint32_t>(transaction.srcSize)
                         << DMA_SxCR_MSIZE_Pos;
        registers->CR |= static_cast<uint32_t>(transaction.dstSize)
                         << DMA_SxCR_PSIZE_Pos;

        if (transaction.srcIncrement)
            registers->CR |= DMA_SxCR_MINC;
        if (transaction.dstIncrement)
            registers->CR |= DMA_SxCR_PINC;

        registers->M0AR = reinterpret_cast<uint32_t>(transaction.srcAddress);
        registers->PAR  = reinterpret_cast<uint32_t>(transaction.dstAddress);
    }
    else
    {
        // In peripheral to memory or memory to memory mode, the source address
        // goes into the peripheral address register

        registers->CR |= static_cast<uint32_t>(transaction.srcSize)
                         << DMA_SxCR_PSIZE_Pos;
        registers->CR |= static_cast<uint32_t>(transaction.dstSize)
                         << DMA_SxCR_MSIZE_Pos;

        if (transaction.srcIncrement)
            registers->CR |= DMA_SxCR_PINC;
        if (transaction.dstIncrement)
            registers->CR |= DMA_SxCR_MINC;

        registers->PAR  = reinterpret_cast<uint32_t>(transaction.srcAddress);
        registers->M0AR = reinterpret_cast<uint32_t>(transaction.dstAddress);
    }

    if (transaction.doubleBufferMode)
    {
        registers->CR |= DMA_SxCR_DBM;
        registers->M1AR =
            reinterpret_cast<uint32_t>(transaction.secondMemoryAddress);
    }

    bool enableInterrupt = false;
    if (transaction.enableHalfTransferInterrupt)
    {
        clearHalfTransferFlag();
        registers->CR |= DMA_SxCR_HTIE;
        enableInterrupt = true;
    }
    if (transaction.enableTransferCompleteInterrupt)
    {
        clearTransferCompleteFlag();
        registers->CR |= DMA_SxCR_TCIE;
        enableInterrupt = true;
    }
    if (transaction.enableTransferErrorInterrupt)
    {
        clearTransferErrorFlag();
        registers->CR |= DMA_SxCR_TEIE;
        enableInterrupt = true;
    }
    if (transaction.enableFifoErrorInterrupt)
    {
        clearFifoErrorFlag();
        registers->CR |= DMA_SxFCR_FEIE;
        enableInterrupt = true;
    }
    if (transaction.enableDirectModeErrorInterrupt)
    {
        clearDirectModeErrorFlag();
        registers->CR |= DMA_SxCR_DMEIE;
        enableInterrupt = true;
    }

    if (enableInterrupt)
    {
        NVIC_SetPriority(irqNumber, 8);
        NVIC_ClearPendingIRQ(irqNumber);
        NVIC_EnableIRQ(irqNumber);
    }
    else
    {
        NVIC_DisableIRQ(irqNumber);
    }
}

void DMAStream::enable()
{
    // Reset all saved flags
    halfTransferFlag     = false;
    transferCompleteFlag = false;
    transferErrorFlag    = false;
    fifoErrorFlag        = false;
    directModeErrorFlag  = false;

    // Before setting EN bit to '1' to start a new transfer, the event
    //  flags corresponding to the stream in DMA_LISR or DMA_HISR
    //  register must be cleared.
    clearAllFlags();

    // Enable the peripheral
    registers->CR |= DMA_SxCR_EN;
}

void DMAStream::disable() { registers->CR &= ~DMA_SxCR_EN; }

void DMAStream::waitForHalfTransfer()
{
    waitForInterruptEventImpl(
        currentSetup.enableHalfTransferInterrupt,
        std::bind(&DMAStream::getHalfTransferFlagStatus, this),
        std::bind(&DMAStream::clearHalfTransferFlag, this), halfTransferFlag,
        -1);
}

void DMAStream::waitForTransferComplete()
{
    waitForInterruptEventImpl(
        currentSetup.enableTransferCompleteInterrupt,
        std::bind(&DMAStream::getTransferCompleteFlagStatus, this),
        std::bind(&DMAStream::clearTransferCompleteFlag, this),
        transferCompleteFlag, -1);
}

bool DMAStream::timedWaitForHalfTransfer(uint64_t timeout_ns)
{
    return waitForInterruptEventImpl(
        currentSetup.enableHalfTransferInterrupt,
        std::bind(&DMAStream::getHalfTransferFlagStatus, this),
        std::bind(&DMAStream::clearHalfTransferFlag, this), halfTransferFlag,
        timeout_ns);
}

bool DMAStream::timedWaitForTransferComplete(uint64_t timeout_ns)
{
    return waitForInterruptEventImpl(
        currentSetup.enableTransferCompleteInterrupt,
        std::bind(&DMAStream::getTransferCompleteFlagStatus, this),
        std::bind(&DMAStream::clearTransferCompleteFlag, this),
        transferCompleteFlag, timeout_ns);
}

void DMAStream::setHalfTransferCallback(std::function<void()> callback)
{
    halfTransferCallback = callback;
}

void DMAStream::resetHalfTransferCallback() { halfTransferCallback = nullptr; }

void DMAStream::setTransferCompleteCallback(std::function<void()> callback)
{
    transferCompleteCallback = callback;
}

void DMAStream::resetTransferCompleteCallback()
{
    transferCompleteCallback = nullptr;
}

void DMAStream::setErrorCallback(std::function<void()> callback)
{
    errorCallback = callback;
}

void DMAStream::resetErrorCallback() { errorCallback = nullptr; }

void DMAStream::readFlags()
{
    uint8_t flags = *ISR >> IFindex;

    halfTransferFlag     = flags & DMA_LISR_HTIF0;
    transferCompleteFlag = flags & DMA_LISR_TCIF0;
    transferErrorFlag    = flags & DMA_LISR_TEIF0;
    fifoErrorFlag        = flags & DMA_LISR_DMEIF0;
    directModeErrorFlag  = flags & DMA_LISR_DMEIF0;
}

int DMAStream::getCurrentBufferNumber()
{
    return (registers->CR & DMA_SxCR_CT) != 0 ? 2 : 1;
}

DMAStream::DMAStream(DMAStreamId id) : id(id)
{
    // Get the channel registers base address and the interrupt flags clear
    // register address
    if (id < DMAStreamId::DMA2_Str0)
    {
        registers = reinterpret_cast<DMA_Stream_TypeDef*>(
            DMA1_BASE + 0x10 + 0x18 * static_cast<uint8_t>(id));

        if (id < DMAStreamId::DMA1_Str4)
        {
            // Streams from 0 to 3 use low registers (LIFCR and LISR)
            IFCR = &DMA1->LIFCR;
            ISR  = &DMA1->LISR;
        }
        else
        {
            // Streams from 4 to 7 use high registers (HIFCR and HISR)
            IFCR = &DMA1->HIFCR;
            ISR  = &DMA1->HISR;
        }
    }
    else
    {
        registers = reinterpret_cast<DMA_Stream_TypeDef*>(
            DMA2_BASE + 0x10 + 0x18 * (static_cast<uint8_t>(id) - 8));

        if (id < DMAStreamId::DMA2_Str4)
        {
            // Streams from 0 to 3 use low registers (LIFCR and LISR)
            IFCR = &DMA2->LIFCR;
            ISR  = &DMA2->LISR;
        }
        else
        {
            // Streams from 4 to 7 use high registers (HIFCR and HISR)
            IFCR = &DMA2->HIFCR;
            ISR  = &DMA2->HISR;
        }
    }

    // Compute the index for the interrupt flags clear register
    // Refer to reference manual for the register bits structure
    int offset = static_cast<uint8_t>(id) % 4;
    IFindex    = (offset % 2) * 6 + (offset / 2) * 16;

    // Select the interrupt
    irqNumber = irqNumberMapping[static_cast<uint8_t>(id)];
}

}  // namespace Boardcore
