/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Alberto Nidasio, Fabrizio Monti
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
#include <utils/Debug.h>

#include <map>

using namespace miosix;

/**
 * Here are defined the IRQHandlers for the various streams.
 *
 * The problem is that some of these stream are used
 * by miosix. The corresponding IRQHandlers are already defined
 * in there, causing conflicts.
 * Moreover, the used streams differ from STM32F407xx to
 * STM32F767xx. That's why some streams are available only
 * for a particular board, or none (DMA2_Stream3 is not available
 * at all).
 */

void __attribute__((naked)) DMA1_Stream0_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z20DMA1_Stream0_IRQImplv");
    restoreContext();
}

void __attribute__((used)) DMA1_Stream0_IRQImpl()
{
    Boardcore::DMADriver::instance().IRQhandleInterrupt(
        Boardcore::DMADefs::DMAStreamId::DMA1_Str0);
}

#ifndef STM32F407xx
// This stream is used by miosix for STM32F407xx boards
void __attribute__((naked)) DMA1_Stream1_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z20DMA1_Stream1_IRQImplv");
    restoreContext();
}

void __attribute__((used)) DMA1_Stream1_IRQImpl()
{
    Boardcore::DMADriver::instance().IRQhandleInterrupt(
        Boardcore::DMADefs::DMAStreamId::DMA1_Str1);
}
#endif  // STM32F407xx

void __attribute__((naked)) DMA1_Stream2_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z20DMA1_Stream2_IRQImplv");
    restoreContext();
}

void __attribute__((used)) DMA1_Stream2_IRQImpl()
{
    Boardcore::DMADriver::instance().IRQhandleInterrupt(
        Boardcore::DMADefs::DMAStreamId::DMA1_Str2);
}

#ifndef STM32F407xx
// This stream is used by miosix for STM32F407xx boards
void __attribute__((naked)) DMA1_Stream3_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z20DMA1_Stream3_IRQImplv");
    restoreContext();
}

void __attribute__((used)) DMA1_Stream3_IRQImpl()
{
    Boardcore::DMADriver::instance().IRQhandleInterrupt(
        Boardcore::DMADefs::DMAStreamId::DMA1_Str3);
}
#endif  // STM32F407xx

void __attribute__((naked)) DMA1_Stream4_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z20DMA1_Stream4_IRQImplv");
    restoreContext();
}

void __attribute__((used)) DMA1_Stream4_IRQImpl()
{
    Boardcore::DMADriver::instance().IRQhandleInterrupt(
        Boardcore::DMADefs::DMAStreamId::DMA1_Str4);
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
        Boardcore::DMADefs::DMAStreamId::DMA1_Str5);
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
        Boardcore::DMADefs::DMAStreamId::DMA1_Str6);
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
        Boardcore::DMADefs::DMAStreamId::DMA1_Str7);
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
        Boardcore::DMADefs::DMAStreamId::DMA2_Str0);
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
        Boardcore::DMADefs::DMAStreamId::DMA2_Str1);
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
        Boardcore::DMADefs::DMAStreamId::DMA2_Str2);
}

// This stream is used by miosix both for STM32F407xx
// and STM32F767xx, so it is simply commented out
// void __attribute__((naked)) DMA2_Stream3_IRQHandler()
// {
//     saveContext();
//     asm volatile("bl _Z20DMA2_Stream3_IRQImplv");
//     restoreContext();
// }

// void __attribute__((used)) DMA2_Stream3_IRQImpl()
// {
//     Boardcore::DMADriver::instance().IRQhandleInterrupt(
//         Boardcore::DMADefs::DMAStreamId::DMA2_Str3);
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
        Boardcore::DMADefs::DMAStreamId::DMA2_Str4);
}

#ifndef STM32F767xx
// This stream is used by miosix for STM32F767xx boards
void __attribute__((naked)) DMA2_Stream5_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z20DMA2_Stream5_IRQImplv");
    restoreContext();
}

void __attribute__((used)) DMA2_Stream5_IRQImpl()
{
    Boardcore::DMADriver::instance().IRQhandleInterrupt(
        Boardcore::DMADefs::DMAStreamId::DMA2_Str5);
}
#endif  // STM32F767xx

void __attribute__((naked)) DMA2_Stream6_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z20DMA2_Stream6_IRQImplv");
    restoreContext();
}

void __attribute__((used)) DMA2_Stream6_IRQImpl()
{
    Boardcore::DMADriver::instance().IRQhandleInterrupt(
        Boardcore::DMADefs::DMAStreamId::DMA2_Str6);
}

#ifndef STM32F767xx
// This stream is used by miosix for STM32F767xx boards
void __attribute__((naked)) DMA2_Stream7_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z20DMA2_Stream7_IRQImplv");
    restoreContext();
}

void __attribute__((used)) DMA2_Stream7_IRQImpl()
{
    Boardcore::DMADriver::instance().IRQhandleInterrupt(
        Boardcore::DMADefs::DMAStreamId::DMA2_Str7);
}
#endif  // STM32F767xx

namespace Boardcore
{

void DMADriver::IRQhandleInterrupt(DMADefs::DMAStreamId id)
{
    DMAStream& stream = streams.at(id);

    stream.readFlags();
    stream.clearAllFlags();

    // Run the callbacks if necessary
    if (stream.halfTransferCallback && stream.halfTransferFlag)
        stream.halfTransferCallback();

    if (stream.transferCompleteCallback && stream.transferCompleteFlag)
        stream.transferCompleteCallback();

    if (stream.errorCallback &&
        (stream.transferErrorFlag || stream.fifoErrorFlag ||
         stream.directModeErrorFlag))
    {
        stream.errorCallback();
    }

    // Wakeup the thread if the user is waiting
    if (stream.waitingThread)
        IRQwakeupThread(stream);
}

void DMADriver::IRQwakeupThread(DMAStream& stream)
{
    // Wakeup the waiting thread
    stream.waitingThread->wakeup();

    // If the waiting thread has a higher priority than the current
    // thread then reschedule
    if (stream.waitingThread->IRQgetPriority() >
        miosix::Thread::IRQgetCurrentThread()->IRQgetPriority())
    {
        miosix::Scheduler::IRQfindNextThread();
    }

    // Clear the thread pointer, this way the thread will be sure it is
    // not a spurious wakeup
    stream.waitingThread = nullptr;
}

DMADriver& DMADriver::instance()
{
    static DMADriver instance;
    return instance;
}

bool DMADriver::tryChannel(DMADefs::DMAStreamId id)
{
    Lock<FastMutex> l(mutex);

    // Return true, meaning that the channel is free, only if it is not yet
    // allocated
    return streams.count(id) == 0;
}

DMAStreamGuard DMADriver::acquireStream(DMADefs::DMAStreamId id,
                                        DMADefs::Channel channel,
                                        std::chrono::nanoseconds timeout)
{
    Lock<FastMutex> l(mutex);

    // Wait until the stream is free or the timeout expires
    while (streams.count(id) != 0)
    {
        if (timeout == std::chrono::nanoseconds::zero())
        {
            cv.wait(l);
        }
        else
        {
            auto res = cv.timedWait(l, timeout.count());

            if (res == TimedWaitResult::Timeout)
            {
                // The timeout expired
                return DMAStreamGuard(nullptr);
            }
        }
    }

    // Enable the clock if not already done
    // TODO: Enable DMA1 or DMA2
    // if (streams.size() == 0)
    //     RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    streams.insert(
        std::pair<DMADefs::DMAStreamId, DMAStream>(id, DMAStream(id, channel)));
    return DMAStreamGuard(&(streams.at(id)));
}

DMAStreamGuard DMADriver::acquireStreamForPeripheral(
    DMADefs::Peripherals peripheral, std::chrono::nanoseconds timeout)
{
    const auto availableStreams =
        DMADefs::mapPeripherals.equal_range(peripheral);

    Lock<FastMutex> l(mutex);
    while (true)
    {
        // Iterate through the streams for that peripheral,
        // return the first available
        for (auto it = availableStreams.first; it != availableStreams.second;
             ++it)
        {
            DMADefs::DMAStreamId id  = it->second.first;
            DMADefs::Channel channel = it->second.second;

            if (streams.count(id) == 0)
            {
                // Stream is free
                streams.insert(std::pair<DMADefs::DMAStreamId, DMAStream>(
                    id, DMAStream(id, channel)));
                return DMAStreamGuard(&(streams.at(id)));
            }
        }

        if (timeout == std::chrono::nanoseconds::zero())
        {
            cv.wait(l);
        }
        else
        {
            auto res = cv.timedWait(l, timeout.count());

            if (res == TimedWaitResult::Timeout)
            {
                // The timeout expired
                return DMAStreamGuard(nullptr);
            }
        }
    }
}

void DMADriver::releaseStream(DMADefs::DMAStreamId id)
{
    Lock<FastMutex> l(mutex);

    if (streams.count(id) != 0)
    {
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
    constexpr int resetValue = 0x0f7d0f7d;
    DMA1->HIFCR              = resetValue;
    DMA1->LIFCR              = resetValue;
    DMA2->HIFCR              = resetValue;
    DMA2->LIFCR              = resetValue;
}

void DMAStream::setup(DMATransaction& transaction)
{
    currentSetup = transaction;

    // Reset the configuration
    registers->CR = 0;

    // Wait for the stream to actually be disabled
    while (registers->CR & DMA_SxCR_EN)
        ;

    setChannel(currentChannel);
    registers->CR |= static_cast<uint32_t>(transaction.direction);
    registers->CR |= static_cast<uint32_t>(transaction.priority);
    if (transaction.circularMode)
        registers->CR |= DMA_SxCR_CIRC;

    setNumberOfDataItems(transaction.numberOfDataItems);

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

    // Select the interrupt number
    IRQn_Type irqNumber = DMADefs::irqNumberMapping[static_cast<uint8_t>(id)];
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

#ifdef STM32F767xx
    invalidateCache();
#endif  // STM32F767xx
}

bool DMAStream::timedWaitForHalfTransfer(std::chrono::nanoseconds timeout_ns)
{
    return waitForInterruptEventImpl(
        currentSetup.enableHalfTransferInterrupt,
        std::bind(&DMAStream::getHalfTransferFlagStatus, this),
        std::bind(&DMAStream::clearHalfTransferFlag, this), halfTransferFlag,
        timeout_ns.count());
}

bool DMAStream::timedWaitForTransferComplete(
    std::chrono::nanoseconds timeout_ns)
{
    return waitForInterruptEventImpl(
        currentSetup.enableTransferCompleteInterrupt,
        std::bind(&DMAStream::getTransferCompleteFlagStatus, this),
        std::bind(&DMAStream::clearTransferCompleteFlag, this),
        transferCompleteFlag, timeout_ns.count());

#ifdef STM32F767xx
    invalidateCache();
#endif  // STM32F767xx
}

#ifdef STM32F767xx
void DMAStream::invalidateCache()
{
    /**
     * STM32F7 boards use data cache. Unluckily the dma doesn't
     * trigger the cache refresh.
     * This means that when copying data to ram, the user won't
     * see the result.
     * This method check if cache invalidation is needed, and
     * forces it if necessary.
     *
     * The memory being invalidated must be 32 bytes aligned.
     */

    // If the data was copied from memory to a peripheral there's
    // no need to worry about cache
    if (currentSetup.direction == DMATransaction::Direction::MEM_TO_PER)
        return;

    constexpr uint8_t CACHE_LINE_SIZE = 32;

    // Aligned ptr: round down to the nearest address that is
    // 32 bytes aligned
    uintptr_t alignedPtr =
        (uintptr_t)currentSetup.dstAddress & ~(CACHE_LINE_SIZE - 1);

    // Evaluate how many bytes were added, due to the round down
    uintptr_t diff = (uintptr_t)currentSetup.dstAddress - alignedPtr;

    // Aligned size: compute the amount of bytes being invalidated
    int32_t alignedSize = currentSetup.numberOfDataItems;
    if (currentSetup.dstSize == DMATransaction::DataSize::BITS_16)
        alignedSize *= 2;
    else if (currentSetup.dstSize == DMATransaction::DataSize::BITS_32)
        alignedSize *= 4;
    alignedSize += diff;

    SCB_InvalidateDCache_by_Addr((uint32_t*)alignedPtr, alignedSize);
}
#endif  // STM32F767xx

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

bool DMAStream::setNumberOfDataItems(const uint16_t nBytes)
{
    // Verify that the stream is disabled while doing it
    if ((registers->CR & DMA_SxCR_EN) != 0)
    {
        // Cannot proceed
        return false;
    }

    currentSetup.numberOfDataItems = nBytes;
    registers->NDTR                = nBytes;
    return true;
}

void DMAStream::setChannel(const DMADefs::Channel channel)
{
    registers->CR |= static_cast<uint32_t>(channel);
}

int DMAStream::getCurrentBufferNumber()
{
    return (registers->CR & DMA_SxCR_CT) != 0 ? 2 : 1;
}

DMAStream::DMAStream(DMADefs::DMAStreamId id, DMADefs::Channel channel)
    : id(id), currentChannel(channel)
{
    // Get the channel registers base address and the interrupt flags clear
    // register address
    if (id < DMADefs::DMAStreamId::DMA2_Str0)
    {
        registers = reinterpret_cast<DMA_Stream_TypeDef*>(
            DMA1_BASE + 0x10 + 0x18 * static_cast<uint8_t>(id));

        if (id < DMADefs::DMAStreamId::DMA1_Str4)
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

        if (id < DMADefs::DMAStreamId::DMA2_Str4)
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
}

DMAStream* DMAStreamGuard::operator->()
{
    D(assert((pStream != nullptr) && "pointer is null"));

    return pStream;
}

}  // namespace Boardcore
