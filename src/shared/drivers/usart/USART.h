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

#include <interfaces/arch_registers.h>
#include <miosix.h>
#include <stddef.h>
#include <utils/ClockUtils.h>

#ifndef USE_MOCK_PERIPHERALS
using USARTType = USART_TypeDef;
#else
// TODO: create test utils
#endif

namespace Boardcore
{

/**
 * @brief Driver for STM32 low level USART peripheral.
 *
 * This driver applies to STM32F4 family.
 *
 * The Universal Synchronous and asynchronous Receiver/Transmitter (USART)
 * allows:
 * - UART communications
 */
class USART
{
public:
    enum class Mode : int
    {
        UART
    };

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

    enum class Baudrate : int
    {
        B1200   = 1200,
        B2400   = 2400,
        B9600   = 9600,
        B19200  = 19200,
        B38400  = 38400,
        B57600  = 57600,
        B115200 = 115200,
        B230400 = 230400,
        B460800 = 460800
    };

    /**
     * @brief Automatically enables the peripheral and timer peripheral clock.
     */
    USART(USARTType *usart, Baudrate baudrate);

    void init();

    /**
     * @brief Set the length of the word to 8 or to 9.
     * @param wl WordLength element that represents the length of the word.
     */
    void setWordLength(WordLength wl);

    /**
     * @brief Set the presence of the parity in the data sent.
     * @param pb ParityBit element that represents the presence of the parity
     * bit.
     */
    void setParity(ParityBit pb);

    /**
     * @brief Set the number of stop bits.
     * @param stopBits number of stop bits [1,2].
     */
    void setStopBits(int stopBits);

    /**
     * @brief Set the baudrate in the BRR register.
     * @param pb Baudrate element that represents the baudrate.
     */
    void setBaudrate(Baudrate br);

    /**
     * @brief Sets the Over8 bit
     */
    void setOversampling(bool oversampling);

    void IRQHandleInterrupt();

    /**
     * @brief Blocking read operation
     */
    int read(void *buf, size_t nChars);

    /**
     * @brief Blocking write operation
     */
    void write(void *buf, size_t nChars);

private:
    USARTType *usart;
    miosix::DynUnsyncQueue<char> rxQueue;  ///< Receiving queue
    Baudrate baudrate;
    ParityBit parity      = ParityBit::NO_PARITY;
    WordLength wordLength = WordLength::BIT8;
    int stopBits          = 1;
    bool over8            = false;
    int id                = 1;  ///< can be 1, 2, 3
};

}  // namespace Boardcore