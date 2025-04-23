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

#pragma once

#include <kernel/scheduler/scheduler.h>
#include <kernel/sync.h>

#include <chrono>
#include <functional>
#include <map>

#include "DMADefs.h"

namespace Boardcore
{

/**
 * @brief This is the configuration struct for
 * a DMA transaction.
 */
struct DMATransaction
{
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
class DMAStreamGuard;

/**
 * @brief This class is responsible for streams acquisition,
 * streams release and interrupt handling.
 */
class DMADriver
{
public:
    void IRQhandleInterrupt(DMADefs::DMAStreamId id);

    static DMADriver& instance();

    /**
     * @return True if the stream is not already in use.
     */
    bool tryStream(DMADefs::DMAStreamId id);

    /**
     * @brief Try to acquire the specified stream and initialize it with the
     * correct channel.
     * @param id The id of the stream to be acquired.
     * @param channel The channel used to initialize the stream.
     * @param timeout The maximum time that will be waited, defaults to waiting
     * forever.
     * @return A stream guard that might be valid or not, depending on the
     * outcome of the request.
     */
    DMAStreamGuard acquireStream(
        DMADefs::DMAStreamId id, DMADefs::Channel channel,
        std::chrono::nanoseconds timeout = std::chrono::nanoseconds::zero());

    /**
     * @brief Try to acquire a stream that is connected to the specified
     * peripheral.
     * @param peripheral The wanted peripheral.
     * @param timeout The maximum time that will be waited, defaults to waiting
     * forever.
     * @return A stream guard that might be valid or not, depending on the
     * outcome of the request.
     */
    DMAStreamGuard acquireStreamForPeripheral(
        DMADefs::Peripherals peripheral,
        std::chrono::nanoseconds timeout = std::chrono::nanoseconds::zero());

    void releaseStream(DMADefs::DMAStreamId id);

private:
    DMADriver();

    /**
     * @brief Wakeup the sleeping thread associated to the stream.
     */
    void IRQwakeupThread(DMAStream& stream);

    miosix::FastMutex mutex;
    miosix::ConditionVariable cv;
    std::map<DMADefs::DMAStreamId, DMAStream> streams;

public:
    DMADriver(const DMADriver&)            = delete;
    DMADriver& operator=(const DMADriver&) = delete;
};

/**
 * @brief This class represents the actual DMA stream.
 * It can be used to setup, start and stop DMA transactions.
 */
class DMAStream
{
    friend DMADriver;

public:
    /**
     * @brief Setup the stream with the given configuration.
     */
    void setup(DMATransaction& transaction);

    /**
     * @brief Activate the stream. As soon as the stream is enabled, it
     * serves any DMA request from/to the peripheral connected to the stream.
     */
    void enable();

    /**
     * @brief Stop the DMA transaction (if running).
     * This is equivalent to killing the transaction: DO NOT expect to be able
     * to restart the transaction from where it was interrupted. The work
     * completed up to the call will still be valid.
     * @warning If set, the transfer complete interrupt will be fired.
     */
    void disable();

    /**
     * @brief Wait for the half transfer complete signal.
     * The caller waits for the corresponding interrupt, if enabled.
     * Otherwise it goes to polling mode on the flag.
     * @warning In case cache is used, this method DOES NOT invalidate
     * the cache lines. It is up to the user.
     */
    void waitForHalfTransfer();

    /**
     * @brief Wait for the transfer complete signal.
     * The caller waits for the corresponding interrupt, if enabled.
     * Otherwise it goes to polling mode on the flag.
     * In case cache is used, this method invalidates the
     * cache lines, so that the user can see the memory as is in ram.
     */
    void waitForTransferComplete();

    /**
     * @brief Wait for the half transfer complete signal.
     * The caller waits for the corresponding interrupt, if enabled.
     * Otherwise it goes to polling mode on the flag.
     * @param timeout_ns The maximum time that will be waited.
     * @return True if the event is reached, false if the
     * timeout expired.
     * @warning In case cache is used, this method DOES NOT invalidate
     * the cache lines. It is up to the user.
     */
    bool timedWaitForHalfTransfer(std::chrono::nanoseconds timeout_ns);

    /**
     * @brief Wait for the transfer complete signal.
     * The caller waits for the corresponding interrupt, if enabled.
     * Otherwise it goes to polling mode on the flag.
     * In case cache is used, this method invalidates the
     * cache lines, so that the user can see the memory as is in ram.
     * @param timeout_ns The maximum time that will be waited.
     * @return True if the event is reached, false if the
     * timeout expired.
     */
    bool timedWaitForTransferComplete(std::chrono::nanoseconds timeout_ns);

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
     * @brief Set the number of bytes to be exchanged during a
     * dma transaction. Useful in case you don't want to change
     * the entire configuration. Use while the stream is not
     * enabled.
     * @return True if the operation succeeded, false otherwise.
     */
    bool setNumberOfDataItems(const uint16_t nBytes);

    /**
     * @brief Select the channel to be used by the stream during
     * the transactions.
     */
    void setChannel(const DMADefs::Channel channel);

    /**
     * @brief Returns the last read status of the half transfer flag.
     */
    inline bool getHalfTransferFlagStatus() { return halfTransferFlag; }

    /**
     * @brief Returns the last read status of the transfer complete flag.
     */
    inline bool getTransferCompleteFlagStatus() { return transferCompleteFlag; }

    /**
     * @brief Returns the last read status of the transfer error flag.
     */
    inline bool getTransferErrorFlagStatus() { return transferErrorFlag; }

    /**
     * @brief Returns the last read status of the fifo error flag.
     */
    inline bool getFifoErrorFlagStatus() { return fifoErrorFlag; }

    /**
     * @brief Returns the last read status of the direct mode error flag.
     */
    inline bool getDirectModeErrorFlagStatus() { return directModeErrorFlag; }

    /**
     * @brief Returns the number of the buffer currently in use.
     *
     * @return 1 or 2 depending on the buffer currently in use.
     */
    int getCurrentBufferNumber();

    inline DMADefs::DMAStreamId getStreamId() { return id; }

    inline DMADefs::Channel getCurrentChannel() { return currentChannel; }

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
    DMAStream(DMADefs::DMAStreamId id, DMADefs::Channel channel);

    DMATransaction currentSetup;

    /**
     * @brief Used to determine if the user thread is
     * waiting to be awakened by an interrupt.
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

    const DMADefs::DMAStreamId id;
    DMADefs::Channel currentChannel;
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
                const long long start = miosix::getTime();

                do
                {
                    readFlags();
                } while (!getEventStatus() &&
                         miosix::getTime() - start < timeout_ns);

                result = getEventStatus();
            }
            else
            {
                while (!getEventStatus())
                    readFlags();
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

#ifdef STM32F767xx
    /**
     * @brief In case cache is used and data is written to ram,
     * we have to invalidate cache lines in order to see the
     * updated data. This function verifies if this operation
     * is needed and performs it.
     */
    void invalidateCache();
#endif  // STM32F767xx

public:
    DMAStream(const DMAStream&)            = delete;
    DMAStream& operator=(const DMAStream&) = delete;

    DMAStream(DMAStream&&) noexcept            = default;
    DMAStream& operator=(DMAStream&&) noexcept = default;
};

/**
 * @brief Simple RAII class to handle DMA streams.
 */
class DMAStreamGuard
{
public:
    DMAStreamGuard(DMAStream* ptr) : pStream(ptr) {}

    ~DMAStreamGuard()
    {
        if (pStream != nullptr)
            DMADriver::instance().releaseStream(pStream->getStreamId());
    }

    DMAStreamGuard(const DMAStreamGuard&)            = delete;
    DMAStreamGuard& operator=(const DMAStreamGuard&) = delete;

    DMAStreamGuard(DMAStreamGuard&&) noexcept            = default;
    DMAStreamGuard& operator=(DMAStreamGuard&&) noexcept = default;

    DMAStream* operator->();

    /**
     * @return True if the stream was correctly allocated and
     * is ready to use. False otherwise.
     */
    inline bool isValid() { return pStream != nullptr; }

private:
    DMAStream* const pStream;
};

}  // namespace Boardcore
