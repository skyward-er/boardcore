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

#pragma once

#include <interfaces/arch_registers.h>
#include <kernel/scheduler/scheduler.h>
#include <kernel/sync.h>
#include <utils/TimedPollingFlag.h>

#include <functional>
#include <map>

namespace Boardcore
{

// TODO: remove and find a better solution for stm32f407
#ifndef DMA_SxCR_MSIZE_Pos
#define DMA_SxCR_MSIZE_Pos (13U)
#endif
// TODO: remove and find a better solution for stm32f407
#ifndef DMA_SxCR_PSIZE_Pos
#define DMA_SxCR_PSIZE_Pos (11U)
#endif

enum class DMAStreamId : uint8_t
{
    // TODO: some streams are commented because the
    // corresponding irq handler is already defined
    // by miosix. For now those streams are not usable,
    // decide how to handle this situation.

    DMA1_Str0 = 0,
    // DMA1_Str1 = 1,
    DMA1_Str2 = 2,
    // DMA1_Str3 = 3,
    DMA1_Str4 = 4,
    DMA1_Str5 = 5,
    DMA1_Str6 = 6,
    DMA1_Str7 = 7,
    DMA2_Str0 = 8,
    DMA2_Str1 = 9,
    DMA2_Str2 = 10,
    // DMA2_Str3 = 11,
    DMA2_Str4 = 12,
    DMA2_Str5 = 13,
    DMA2_Str6 = 14,
    DMA2_Str7 = 15,
};

/**
 * @brief Mapping between `DMAStreamId` and the corresponding irq number.
 * This is needed because irq number values are not contiguous and they are
 * architecture dependent.
 */
const IRQn_Type irqNumberMapping[] = {
    DMA1_Stream0_IRQn, DMA1_Stream1_IRQn, DMA1_Stream2_IRQn, DMA1_Stream3_IRQn,
    DMA1_Stream4_IRQn, DMA1_Stream5_IRQn, DMA1_Stream6_IRQn, DMA1_Stream7_IRQn,
    DMA2_Stream0_IRQn, DMA2_Stream1_IRQn, DMA2_Stream2_IRQn, DMA2_Stream3_IRQn,
    DMA2_Stream4_IRQn, DMA2_Stream5_IRQn, DMA2_Stream6_IRQn, DMA2_Stream7_IRQn,
};

struct DMATransaction
{
    enum class Channel : uint32_t
    {
        CHANNEL0 = 0,
        CHANNEL1 = DMA_SxCR_CHSEL_0,
        CHANNEL2 = DMA_SxCR_CHSEL_1,
        CHANNEL3 = DMA_SxCR_CHSEL_1 | DMA_SxCR_CHSEL_0,
        CHANNEL4 = DMA_SxCR_CHSEL_2,
        CHANNEL5 = DMA_SxCR_CHSEL_2 | DMA_SxCR_CHSEL_0,
        CHANNEL6 = DMA_SxCR_CHSEL_2 | DMA_SxCR_CHSEL_1,
        CHANNEL7 = DMA_SxCR_CHSEL,
    };

    enum class Direction : uint16_t
    {
        MEM_TO_MEM = DMA_SxCR_DIR_1,
        MEM_TO_PER = DMA_SxCR_DIR_0,
        PER_TO_MEM = 0,
    };

    enum class Priority : uint32_t
    {
        VERY_HIGH = DMA_SxCR_PL,
        HIGH      = DMA_SxCR_PL_1,
        MEDIUM    = DMA_SxCR_PL_0,
        LOW       = 0,
    };

    enum class DataSize : uint8_t
    {
        BITS_8 = 0,
        BITS_16,
        BITS_32,
    };

    Channel channel                      = Channel::CHANNEL0;
    Direction direction                  = Direction::MEM_TO_MEM;
    Priority priority                    = Priority::LOW;
    DataSize srcSize                     = DataSize::BITS_32;
    DataSize dstSize                     = DataSize::BITS_32;
    volatile void* srcAddress            = nullptr;
    volatile void* dstAddress            = nullptr;
    volatile void* secondMemoryAddress   = nullptr;
    uint16_t numberOfDataItems           = 0;
    bool srcIncrement                    = false;
    bool dstIncrement                    = false;
    bool circularMode                    = false;
    bool doubleBufferMode                = false;
    bool enableHalfTransferInterrupt     = false;
    bool enableTransferCompleteInterrupt = false;
    bool enableTransferErrorInterrupt    = false;
    bool enableFifoErrorInterrupt        = false;
    bool enableDirectModeErrorInterrupt  = false;
};

// Forward declaration
class DMAStream;

class DMADriver
{
public:
    void IRQhandleInterrupt(DMAStreamId id);

    static DMADriver& instance();

    bool tryChannel(DMAStreamId id);

    DMAStream& acquireStream(DMAStreamId id);

    void releaseStream(DMAStreamId id);

private:
    DMADriver();

    void IRQwakeupThread(DMAStream* stream);

    miosix::FastMutex mutex;
    miosix::ConditionVariable cv;
    std::map<DMAStreamId, DMAStream*> streams;

public:
    DMADriver(const DMADriver&)            = delete;
    DMADriver& operator=(const DMADriver&) = delete;
};

class DMAStream
{
    friend DMADriver;

public:
    /**
     * @brief Setup the stream with the given configuration.
     */
    void setup(DMATransaction transaction);

    /**
     * @brief Activate the stream. As soon as the stream is enabled, it
     * can serve any DMA request from the peripheral connected to the stream.
     */
    void enable();

    void disable();

    void waitForHalfTransfer();

    void waitForTransferComplete();

    bool timedWaitForHalfTransfer(uint64_t timeout_ns);

    bool timedWaitForTransferComplete(uint64_t timeout_ns);

    void setHalfTransferCallback(std::function<void()> callback);

    void resetHalfTransferCallback();

    void setTransferCompleteCallback(std::function<void()> callback);

    void resetTransferCompleteCallback();

    void setErrorCallback(std::function<void()> callback);

    void resetErrorCallback();

    /**
     * @brief Reads the current flags status.
     *
     * The values can be read with the get***FlagStatus functions.
     */
    void readFlags();

    /**
     * @brief Returns the last read status of the half transfer flag.
     *
     * TODO: Explain what this flag intails and what to do.
     */
    inline bool getHalfTransferFlagStatus() { return halfTransferFlag; }

    /**
     * @brief Returns the last read status of the transfer complete flag.
     *
     * TODO: Explain what this flag intails and what to do.
     */
    inline bool getTransferCompleteFlagStatus() { return transferCompleteFlag; }

    /**
     * @brief Returns the last read status of the transfer error flag.
     *
     * TODO: Explain what this flag intails and what to do.
     */
    inline bool getTransferErrorFlagStatus() { return transferErrorFlag; }

    /**
     * @brief Returns the last read status of the fifo error flag.
     *
     * TODO: Explain what this flag intails and what to do.
     */
    inline bool getFifoErrorFlagStatus() { return fifoErrorFlag; }

    /**
     * @brief Returns the last read status of the direct mode error flag.
     *
     * TODO: Explain what this flag intails and what to do.
     */
    inline bool getDirectModeErrorFlagStatus() { return directModeErrorFlag; }

    /**
     * @brief Returns the number of the buffer currently in use.
     *
     * @return 1 or 2 depending on the buffer currently in use.
     */
    int getCurrentBufferNumber();

    inline void clearHalfTransferFlag()
    {
        *IFCR |= DMA_LIFCR_CHTIF0 << IFindex;
    }

    inline void clearTransferCompleteFlag()
    {
        *IFCR |= DMA_LIFCR_CTCIF0 << IFindex;
    }

    inline void clearTransferErrorFlag()
    {
        *IFCR |= DMA_LIFCR_CTEIF0 << IFindex;
    }

    inline void clearFifoErrorFlag() { *IFCR |= DMA_LIFCR_CFEIF0 << IFindex; }

    inline void clearDirectModeErrorFlag()
    {
        *IFCR |= DMA_LIFCR_CDMEIF0 << IFindex;
    }

    /**
     * @brief Clear all the flags for the selected stream in the DMA ISR
     * register (LISR or HISR depending on the selected stream id).
     */
    inline void clearAllFlags()
    {
        *IFCR |= (DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTCIF0 | DMA_LIFCR_CTEIF0 |
                  DMA_LIFCR_CFEIF0 | DMA_LIFCR_CDMEIF0)
                 << IFindex;
    }

private:
    DMAStream(DMAStreamId id);

    DMATransaction currentSetup;

    /**
     * @brief Used to determine if the user thread is
     * waiting to be awaked by an interrupt.
     */
    miosix::Thread* waitingThread = nullptr;

    // These flags are set by the interrupt routine and tells the user
    // which event were triggered
    bool halfTransferFlag     = false;
    bool transferCompleteFlag = false;
    bool transferErrorFlag    = false;
    bool fifoErrorFlag        = false;
    bool directModeErrorFlag  = false;

    std::function<void()> halfTransferCallback;
    std::function<void()> transferCompleteCallback;
    std::function<void()> errorCallback;

    DMAStreamId id;
    IRQn_Type irqNumber;
    DMA_Stream_TypeDef* registers;

    volatile uint32_t* ISR;   ///< Interrupt status register
    volatile uint32_t* IFCR;  ///< Interrupt flags clear register
    int IFindex;              ///< Interrupt flags index

    inline bool waitForInterruptEventImpl(
        bool isInterruptEnabled, std::function<bool()> getEventStatus,
        std::function<void()> clearEventStatus, bool& eventTriggered,
        long long timeout_ns)
    {
        // Return value: true if the event was triggered, false if the timeout
        // expired
        bool result = false;

        // If the interrupt is enabled we can just pause and wait for it.
        // Otherwise we need to pool the flag.
        if (isInterruptEnabled)
        {
            // Here we have 2 cases:
            // - This function has been called after the interrupt fired. In
            // this case the interrupt saves the flags status and we check them
            // - The interrupt has not yet fired. We pause the thread and the
            // interrupt will wake us up

            if (eventTriggered)
            {
                result = true;
            }
            else
            {
                // Save the current thread pointer
                waitingThread = miosix::Thread::getCurrentThread();

                // Wait until the thread is woken up and the pointer is cleared
                miosix::FastInterruptDisableLock dLock;
                if (timeout_ns >= 0)
                {
                    do
                    {
                        if (miosix::Thread::IRQenableIrqAndTimedWait(
                                dLock, timeout_ns + miosix::getTime()) ==
                            miosix::TimedWaitResult::Timeout)
                        {
                            result = false;

                            // If the timeout expired we clear the thread
                            // pointer so that the interrupt, if it will occur,
                            // will not wake up the thread (and we can exit the
                            // while loop)
                            waitingThread = nullptr;
                        }
                        else
                        {
                            result = true;
                        }
                    } while (waitingThread);
                }
                else
                {
                    do
                    {
                        miosix::Thread::IRQenableIrqAndWait(dLock);
                    } while (waitingThread);
                    result = true;
                }
            }

            // Before returning we need to clear the flags otherwise we could
            // get misfires
            eventTriggered = false;
        }
        else
        {
            // Pool the flag if the user did not enable the interrupt
            if (timeout_ns >= 0)
            {
                // TODO: this doesn't work, because the methods passed as
                // getEventStatus do not update the flags (reading them
                // by calling readFlags()).
                // This means that it always polls a value that will
                // never become true, then exit when the timeout is
                // reached.
                result = timedPollingFlag(getEventStatus, timeout_ns);
            }
            else
            {
                while (!getEventStatus())
                {
                    readFlags();
                }
                result = true;
            }

            if (result)
            {
                // Clear the flag
                clearEventStatus();
            }
        }

        return result;
    }

public:
    DMAStream(const DMAStream&)            = delete;
    DMAStream& operator=(const DMAStream&) = delete;
};

}  // namespace Boardcore
