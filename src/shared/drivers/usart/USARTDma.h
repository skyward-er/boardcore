/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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

#include <diagnostic/PrintLogger.h>
#include <drivers/dma/DMA.h>
#include <drivers/usart/USART.h>
#include <interfaces/arch_registers.h>
#include <miosix.h>
#include <utils/ClockUtils.h>

#include <chrono>
#include <string>

#include "arch/common/drivers/serial.h"

// #ifndef USE_MOCK_PERIPHERALS
// using USARTType = USART_TypeDef;
// #else
// // TODO: Create test utils
// using USARTType = USART_TypeDef;
// #endif

// #if defined(UART8)
// #define N_USART_PORTS 8
// #elif defined(UART7)
// #define N_USART_PORTS 7
// #elif defined(USART6)
// #define N_USART_PORTS 6
// #elif defined(UART5)
// #define N_USART_PORTS 5
// #elif defined(UART4)
// #define N_USART_PORTS 4
// #elif defined(USART3)
// #define N_USART_PORTS 3
// #elif defined(USART2)
// #define N_USART_PORTS 2
// #elif defined(USART1)
// #define N_USART_PORTS 1
// #else
// #error "Your architecture doesn't support UART"
// #endif

namespace Boardcore
{

/**
 * @brief Driver for STM32F4 low level USART/UART peripheral.
 *
 * It allows to configure some low level parameters such as word length, parity,
 * stop bits and oversampling.
 */
class USARTDma : public USARTInterface
{
public:
    enum class WordLength : bool
    {
        BIT8 = 0,
        BIT9 = 1
    };

    enum class ParityBit : bool
    {
        NO_PARITY = 0,
        PARITY    = 1
    };

    /**
     * @brief Interrupt handler that deals with receive and idle interrupts.
     *
     * Used to implement the reads, fills up a buffer when the interrupt is
     * fired.
     */
    // void IRQhandleInterrupt();

    /**
     * @brief Automatically enables the peripheral and timer peripheral clock.
     *
     * Sets the default values for all the parameters (1 stop bit, 8 bit data,
     * no control flow and no oversampling).
     * Requires that the gpios are configured for the choosen uart.
     *
     * @note An internal queue is used to store the received data, before the
     * user reads it. Reading more data than the queue capacity is allowed, but
     * only up to the internal queue capacity will be returned.
     *
     * @param usart structure that represents the usart peripheral [accepted
     * are: USART1, USART2, USART3, UART4, UART5, USART6, UART7, UART8].
     * @param baudrate Baudrate in bit per second. Default values are [2400,
     * 9600, 19200, 38400, 57600, 115200, 230400, 256000, 460800, 921600]
     * @param queueLen Length of the internal queue for the received data.
     *
     * TODO: add missing data
     */
    USARTDma(USARTType* usart, int baudrate, DMAStreamGuard* rxStream,
          DMAStreamGuard* txStream,
          unsigned int queueLen = INTERNAL_QUEUE_LENGTH);

    // void disableDma() { useDma = false; }
    // bool enableDma()
    // {
    //     useDma = dmaRxStream != nullptr && dmaTxStream != nullptr;
    //     return useDma;
    // }
    void disableDma();
    bool enableDma();

    ///< Delete copy/move constructors/operators.
    USARTDma(const USARTDma&)            = delete;
    USARTDma& operator=(const USARTDma&) = delete;
    USARTDma(USARTDma&&)                 = delete;
    USARTDma& operator=(USARTDma&&)      = delete;

    /**
     * @brief Disables the flags for the generation of the interrupts, the IRQ
     * from the NVIC, the peripheral and removes his pointer from the ports
     * list.
     */
    ~USARTDma() override;

    /**
     * @brief Non-blocking read operation to read nBytes or till the data
     * transfer is complete.
     * @warning could also read 0 bytes.
     *
     * @param buffer Buffer that will contain the received data.
     * @param nBytes Maximum size of the buffer.
     * @return If operation succeeded.
     */
    [[nodiscard]] bool read(void* buffer, size_t nBytes)
    {
        size_t temp;
        auto timeout = std::chrono::nanoseconds::zero();
        return readImpl(buffer, nBytes, temp, false, timeout);
    }

    /**
     * @brief Non-blocking read operation to read nBytes or till the data
     * transfer is complete.
     * @warning could also read 0 bytes.
     *
     * @param buffer Buffer that will contain the received data.
     * @param nBytes Maximum size of the buffer.
     * @param nBytesRead Number of bytes read.
     * @return If operation succeeded.
     */
    [[nodiscard]] bool read(void* buffer, size_t nBytes, size_t& nBytesRead)
    {
        auto timeout = std::chrono::nanoseconds::zero();
        return readImpl(buffer, nBytes, nBytesRead, false, timeout);
    };

    /**
     * @brief Blocking write operation.
     * @param buffer Buffer that contains the data to be sent.
     * @param nBytes Bytes to be sent.
     */
    void write(const void* buf, size_t nBytes);

    /**
     * @brief Write a string to the serial, comprising the '\0' character.
     * @param buffer Buffer that contains the string to be sent.
     */
    void writeString(const char* buffer);

    /**
     * @brief Given a filename, uses the USART interface to stream the file in
     * 1KB chunks
     * @param fileName String containing the name of the file to stream.
     * @return A boolean value (true if stream is complete, false otherwise).
     */
    // bool writeFile(const std::string& fileName);

    /**
     * @brief Set the length of the word to 8 or to 9.
     *
     * @param wl WordLength element that represents the length of the word.
     */
    void setWordLength(WordLength wl);

    /**
     * @brief Set the presence of the parity in the data sent.
     *
     * @param pb ParityBit element that represents the presence of the parity
     * bit.
     */
    void setParity(ParityBit pb);

    /**
     * @brief Set the number of stop bits.
     *
     * @param stopBits number of stop bits [1,2].
     */
    void setStopBits(int stopBits);

    /**
     * @brief Set the baudrate in the BRR register.
     *
     * @param baudrate Baudrate in bit per second. Default values are [2400,
     * 9600, 19200, 38400, 57600, 115200, 230400, 256000, 460800, 921600]
     */
    void setBaudrate(int baudrate);

    /**
     * @brief Sets the Over8 bit.
     *
     * If it is set, the speed is increased; If it is reset the tolerance is
     * increased.
     */
    void setOversampling(bool oversampling);

    /**
     * @brief Clears the rxQueue.
     */
    void clearQueue() {} // non fa nulla, solo per esporla (TODO: fix)

private:
    /**
     * @brief Read method implementation that supports both
     * blocking/non-blocking mode and the return of the number of bytes read.
     * @param buffer Buffer that will contain the received data.
     * @param nBytes Maximum size of the buffer.
     * @param nBytesRead Number of bytes read.
     * @param blocking Whether the read should block or not; in case it isn't
     * blocking the read could return also 0 bytes.
     * @param timeout The maximum time that will be waited when in blocking
     * mode, 0 to disable the timeout and wait forever.
     * @return Whether bytes were read and no timeout occurred.
     */
    [[nodiscard]] bool readImpl(void* buffer, size_t nBytes, size_t& nBytesRead,
                                const bool blocking,
                                std::chrono::nanoseconds timeout) override;
    
    bool readImplDma(void* buffer, uint16_t nBytes, std::chrono::nanoseconds timeout);

    miosix::FastMutex rxMutex;  ///< mutex for receiving on serial
    miosix::FastMutex txMutex;  ///< mutex for transmitting on serial

    miosix::Thread* rxWaiter =
        nullptr;  ///< The thread that is waiting to receive data

    // miosix::DynUnsyncQueue<char> rxQueue;  ///< Receiving queue
    bool idle             = true;          ///< Receiver idle
    ParityBit parity      = ParityBit::NO_PARITY;
    WordLength wordLength = WordLength::BIT8;
    int stopBits          = 1;      ///< Number of stop bits [1,2]
    bool over8            = false;  ///< Oversalmpling 8 bit
    bool error            = false;  ///< Error occurred while receiving message

    ///< Default queue length
    static constexpr unsigned int INTERNAL_QUEUE_LENGTH = 256;

    DMAStreamGuard* const dmaRxStream;
    DMAStreamGuard* const dmaTxStream;
    bool useDma = false;
};

}  // namespace Boardcore
