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

#include <interfaces/arch_registers.h>

/**
 * @brief Driver for STM32 DMA streams.
 *
 * Direct Memory Access is used in order to provide high-speed data transfer
 * between peripherals and memory and betweeen memory and memory. Data can be
 * quickly moved by DMA without any CPU interacion.
 *
 * The STM32F4 family features two DMA controller, each with 8 streams.
 *
 * DMA features are:
 * - Four-word depth 32 FIFO memory buffers per stream, that can be used in FIFO
 * mode or direct mode
 * - Double buffering
 * - Software programmable piorities between DMA stream requests (4 levels)
 * - The number of data items to be transferred can be managed either by the DMA
 * controller or by the peripheral:
 *   - DMA flow controller: the number of data items to be transferred is
 * software-programmable form 1 to 65535
 *   - Peripheral flow controller: the number of data items to be transferred is
 * unknown and controlled by the source or the destination peripheral that
 * signals the end of the transfer by hardware
 * - Incrementing or non incrementing adressing for source and destination
 * - Circular buffer management
 *
 * Note that only DMA2 controller can perform memory-to-memory transactions.
 *
 * Each DMA transfer consists of three operations:
 * - A loading from the peripheral data register or a location in memory
 * - A storage of the data loaded to the peripheral data register or a location
 * in memory
 * - A post-decrement of the counter of transeffer data items (NDTR)
 *
 * After an event, the peripheral sends a request signal to the DMA controller.
 * The DMA controller serves the request depending on the channel priorities. As
 * soon as the DMA controller accesses the peripheral, an acknowledge signal is
 * sent to the peripheral by the DMA controller. The peripheral releases its
 * request as soon as it gets the acknowledge signal.
 *
 * Each stream is associated with a DMA request that can be selected out of 8
 * possible channel requests.
 *
 * Note that configuration functions have effect only if the stream is disabled.
 */
class DMAStream
{
public:
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

    enum class MemoryBurstConfiguration : uint32_t
    {
        SINGLE_TRANSFER = 0,
        INCR4           = DMA_SxCR_MBURST_0,
        INCR8           = DMA_SxCR_MBURST_1,
        INCR16          = DMA_SxCR_MBURST,
    };

    enum class PeripheralBurstConfiguration : uint32_t
    {
        SINGLE_TRANSFER = 0,
        INCR4           = DMA_SxCR_PBURST_0,
        INCR8           = DMA_SxCR_PBURST_1,
        INCR16          = DMA_SxCR_PBURST,
    };

    enum class PriorityLevel : uint32_t
    {
        LOW       = 0,
        MEDIUM    = DMA_SxCR_PL_0,
        HIGH      = DMA_SxCR_PL_1,
        VERY_HIGH = DMA_SxCR_PL,
    };

    enum class MemoryDataSize : uint32_t
    {
        BYTE      = 0,
        HALF_WORD = DMA_SxCR_MSIZE_0,
        WORD      = DMA_SxCR_MSIZE_1
    };

    enum class PeripheralDataSize : uint32_t
    {
        BYTE      = 0,
        HALF_WORD = DMA_SxCR_PSIZE_0,
        WORD      = DMA_SxCR_PSIZE_1
    };

    enum class DataTransferDirection : uint32_t
    {
        PERIPH_TO_MEM = 0,
        MEM_TO_PERIPH = DMA_SxCR_DIR_0,
        MEM_TO_MEM    = DMA_SxCR_DIR_1,
    };

    DMAStream(DMA_Stream_TypeDef *dmaStream);

    void reset();

    void enable();

    void disable();

    void setStreamChannel(Channel channel);

    void setStreamMemoryBurstConfiguration(MemoryBurstConfiguration config);

    void setStreamPeripheralBurstConfiguration(
        PeripheralBurstConfiguration config);

    void enableDoubleBufferMode();

    void disableDoubleBufferMode();

    void setStreamPriorityLevel(PriorityLevel priorityLevel);

    void setMemoryDataSize(MemoryDataSize size);

    void setPeripheralDataSize(PeripheralDataSize size);

    void enableMemoryIncrement();

    void disableMemoryIncrement();

    void enablePeripheralIncrement();

    void disablePeripheralIncrement();

    void enableCircularMode();

    void disableCircularMode();

    void setDataTransferDirection(DataTransferDirection direction);

    void setPeripheralFlowController();

    void setDMAFlowController();

    void disableTransferCompleteInterrupt();

    void enableTransferCompleteInterrupt();

    void enableHalfTransferCompleteInterrupt();

    void disableHalfTransferCompleteInterrupt();

    void enableTransferErrorInterrupt();

    void disableTransferErrorInterrupt();

    void enableDirectModeErrorInterrupt();

    void disableDirectModeErrorInterrupt();

    /**
     * @brief Sets the number of data items to transfer.
     *
     * The NDTR register decrements after each DMA transfer. Once the transfer
     * has completed, this register can either stay at zero (when the stream is
     * in normal mode) or be reloaded automatically with the previusly
     * programmed value in the following cases:
     * - When the stream is configure in circular mode
     * - When the stream is enabled again (by setting EN bit to 1)
     * If the value of the NDTR register is zero, no transaction can be served
     * even if the stream is enabled.
     */
    void setNumberOfDataItems(uint16_t numberOfDataItems);

    uint16_t readNumberOfDataItems();

    void setPeripheralAddress(uint32_t address);

    void setMemory0Address(uint32_t address);

    /**
     * @brief Sets the memory address used only in double buffer mode.
     */
    void setMemory1Address(uint32_t address);

private:
    DMA_Stream_TypeDef *dmaStream;
}

inline DMAStream::DMAStream(DMA_Stream_TypeDef *dmaStream)
    : dmaStream(dmaStream)
{
}

inline void DMAStream::reset() { dmaStream->CR = 0; }

inline void DMAStream::enable() { dmaStream->CR |= DMA_SxCR_EN; }

inline void DMAStream::disable() { dmaStream->CR &= ~DMA_SxCR_EN; }

inline void DMAStream::setStreamChannel(Channel channel)
{
    // First clear the configuration
    dmaStream->CR &= ~DMA_SxCR_CHSEL;

    // Set the new value
    dmaStream->CR |= (uint32_t)channel;
}

inline void DMAStream::setStreamMemoryBurstConfiguration(
    MemoryBurstConfiguration config)
{
    // First clear the configuration
    dmaStream->CR &= ~DMA_SxCR_MBURST;

    // Set the new value
    dmaStream->CR |= (uint32_t)config;
}

inline void DMAStream::setStreamPeripheralBurstConfiguration(
    PeripheralBurstConfiguration config)
{
    // First clear the configuration
    dmaStream->CR &= ~DMA_SxCR_PBURST;

    // Set the new value
    dmaStream->CR |= (uint32_t)config;
}

inline void DMAStream::enableDoubleBufferMode()
{
    dmaStream->CR |= DMA_SxCR_DBM;
}

inline void DMAStream::disableDoubleBufferMode()
{
    dmaStream->CR &= ~DMA_SxCR_DBM;
}

inline void DMAStream::setStreamPriorityLevel(PriorityLevel priorityLevel)
{
    // First clear the configuration
    dmaStream->CR &= ~DMA_SxCR_PL;

    // Set the new value
    dmaStream->CR |= (uint32_t)priorityLevel;
}

inline void DMAStream::setMemoryDataSize(MemoryDataSize size)
{
    // First clear the configuration
    dmaStream->CR &= ~DMA_SxCR_MSIZE;

    // Set the new value
    dmaStream->CR |= (uint32_t)size;
}

inline void DMAStream::enableMemoryIncrement()
{
    dmaStream->CR |= DMA_SxCR_MINC;
}

inline void DMAStream::disableMemoryIncrement()
{
    dmaStream->CR &= ~DMA_SxCR_MINC;
}

inline void DMAStream::enablePeripheralIncrement()
{
    dmaStream->CR |= DMA_SxCR_PINC;
}

inline void DMAStream::disablePeripheralIncrement()
{
    dmaStream->CR &= ~DMA_SxCR_PINC;
}

inline void DMAStream::enableCircularMode() { dmaStream->CR |= DMA_SxCR_CIRC; }

inline void DMAStream::disableCircularMode() { dmaStream->CR |= DMA_SxCR_CIRC; }

inline void DMAStream::setDataTransferDirection(DataTransferDirection direction)
{
    // First clear the configuration
    dmaStream->CR &= ~DMA_SxCR_DIR;

    // Set the new value
    dmaStream->CR |= (uint32_t)direction;
}

inline void DMAStream::setPeripheralFlowController()
{
    dmaStream->CR |= DMA_SxCR_PFCTRL;
}

inline void DMAStream::setDMAFlowController()
{
    dmaStream->CR &= ~DMA_SxCR_PFCTRL;
}

inline void DMAStream::disableTransferCompleteInterrupt()
{
    dmaStream->CR |= DMA_SxCR_TCIE;
}

inline void DMAStream::enableTransferCompleteInterrupt()
{
    dmaStream->CR &= ~DMA_SxCR_TCIE;
}

inline void DMAStream::enableHalfTransferCompleteInterrupt()
{
    dmaStream->CR |= DMA_SxCR_HTIE;
}

inline void DMAStream::disableHalfTransferCompleteInterrupt()
{
    dmaStream->CR &= ~DMA_SxCR_HTIE;
}

inline void DMAStream::enableTransferErrorInterrupt()
{
    dmaStream->CR |= DMA_SxCR_TEIE;
}

inline void DMAStream::disableTransferErrorInterrupt()
{
    dmaStream->CR &= ~DMA_SxCR_TEIE;
}

inline void DMAStream::enableDirectModeErrorInterrupt()
{
    dmaStream->CR |= DMA_SxCR_DMEIE;
}

inline void DMAStream::disableDirectModeErrorInterrupt()
{
    dmaStream->CR &= ~DMA_SxCR_DMEIE;
}

inline void DMAStream::setNumberOfDataItems(uint16_t numberOfDataItems)
{
    dmaStream->NDTR = numberOfDataItems;
}

inline uint16_t DMAStream::readNumberOfDataItems() { return dmaStream->NDTR; }

inline void DMAStream::setPeripheralAddress(uint32_t address)
{
    dmaStream->PAR = address;
}

inline void DMAStream::setMemory0Address(uint32_t address)
{
    dmaStream->M0AR = address;
}

inline void DMAStream::setMemory1Address(uint32_t address)
{
    dmaStream->M1AR = address;
}
