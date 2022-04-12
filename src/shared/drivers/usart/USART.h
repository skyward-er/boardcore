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
#include <utils/ClockUtils.h>

#include <string>

#include "arch/common/drivers/serial.h"

#ifndef USE_MOCK_PERIPHERALS
using USARTType = USART_TypeDef;
#else
// TODO: create test utils
#endif

namespace Boardcore
{
/**
 * @brief Abstract class that implements the interface for the serial
 * communication
 */
class USARTInterface
{
public:
    enum class Baudrate : int
    {
        // B1200   = 1200, // NOT WORKING WITH 1200 baud
        B2400   = 2400,
        B9600   = 9600,
        B19200  = 19200,
        B38400  = 38400,
        B57600  = 57600,
        B115200 = 115200,
        B230400 = 230400,
        B460800 = 460800,
        B921600 = 921600
    };

    /**
     * @brief interrupt handler that deals with receive interrupt, idle
     * interrupt
     */
    virtual void IRQhandleInterrupt() = 0;

    virtual ~USARTInterface() = 0;

    /**
     * @brief initializes the peripheral enabling his interrupts, enabling the
     * interrupts in the NVIC and setting the pins with the appropriate
     * alternate functions. All the setup phase must be done before the
     * initialization of the peripheral. Initializes also the serial port using
     * the chosen pins.
     */
    virtual bool init() = 0;

    /**
     * @brief Blocking read operation to read nBytes or till the data transfer
     * is complete
     */
    virtual int read(void *buffer, size_t nBytes) = 0;

    /**
     * @brief Blocking write operation
     */
    virtual int write(void *buf, size_t nChars) = 0;

    /**
     * @brief Write a string to the serial, comprising the '\0' character
     */
    virtual int writeString(const char *buffer) = 0;

    /**
     * @brief returns the id of the serial
     */
    int getId() { return id; };

protected:
    USARTType
        *usart;  ///< pointer to the struct representing the USART peripheral
    int id = 1;  ///< can be 1, 2, 3
    bool initialized =
        false;  ///< True if init() already called successfully, false otherwise
    Baudrate baudrate;  ///< Baudrate of the serial communication
};

/**
 * @brief Driver for STM32 low level USART peripheral.
 *
 * This driver applies to STM32F4 family.
 *
 * The Universal Synchronous and asynchronous Receiver/Transmitter (USART)
 * allows:
 * - UART communications
 */
class USART : public USARTInterface
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
     * @brief interrupt handler that deals with receive interrupt, idle
     * interrupt
     */
    void IRQhandleInterrupt();

    /**
     * @brief Automatically enables the peripheral and timer peripheral clock.
     * sets the default values for all the parameters (1 stop bit, 8 bit data,
     * no control flow and no oversampling). Initializes the serial port using
     * the default pins, which are:
     * - USART1: tx=PA9  rx=PA10 cts=PA11 rts=PA12
     * - USART2: tx=PA2  rx=PA3  cts=PA0  rts=PA1
     * - USART3: tx=PB10 rx=PB11 cts=PB13 rts=PB14
     * @param usart structure that represents the usart peripheral [accepted
     * are: USART1, USART2 or USART3].
     * @param baudrate member of the enum Baudrate that represents the baudrate
     * with which the communication will take place.
     */
    USART(USARTType *usart, Baudrate baudrate);

    ~USART();

    /**
     * @brief initializes the peripheral enabling his interrupts, enabling the
     * interrupts in the NVIC and setting the pins with the appropriate
     * alternate functions. All the setup phase must be done before the
     * initialization of the peripheral. The pins must be initialized before
     * calling this function.
     */
    bool init();

    /**
     * @brief initializes the pins with the appropriate alternate functions
     * @param tx trasmission pin
     * @param nAFtx trasmission pin alternate function
     * @param rx reception pin
     * @param nAFrx reception pin alternate function
     */
    bool initPins(miosix::GpioPin tx, int nAFtx, miosix::GpioPin rx, int nAFrx);

    /**
     * @brief Blocking read operation to read nBytes or till the data transfer
     * is complete
     */
    int read(void *buffer, size_t nBytes);

    /**
     * @brief Blocking write operation
     */
    int write(void *buf, size_t nChars);

    /**
     * @brief Write a string to the serial, comprising the '\0' character
     */
    int writeString(const char *buffer);

    void enableDMA();

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
     * @brief Sets the Over8 bit. If it is set, the speed is increased; if it is
     * reset the tolerance is increased.
     */
    void setOversampling(bool oversampling);

    /**
     * @brief clears the rxQueue
     */
    void clearQueue();

    /// Pointer to serial port classes to let interrupts access the classes
    static USART *ports[];

private:
    bool pinInitialized = false;
    IRQn_Type irqn;
    miosix::FastMutex rxMutex;  ///< mutex for receiving on serial
    miosix::FastMutex txMutex;  ///< mutex for transmitting on serial
    miosix::Thread *rxWaiting =
        0;  ///< pointer to the waiting on receive thread
    miosix::DynUnsyncQueue<char> rxQueue;  ///< Receiving queue
    bool idle             = true;          ///< Receiver idle
    ParityBit parity      = ParityBit::NO_PARITY;
    WordLength wordLength = WordLength::BIT8;
    int stopBits          = 1;      ///< number of stop bits [1,2]
    bool over8            = false;  ///< oversalmpling 8 bit
};

/**
 * @brief Wrapper for the STM32Serial driver in miosix.
 */
class STM32SerialWrapper : public USARTInterface
{
public:
    /**
     * @brief interrupt handler that deals with receive interrupt, idle
     * interrupt
     */
    void IRQhandleInterrupt();

    STM32SerialWrapper(USARTType *usart, Baudrate baudrate);

    ~STM32SerialWrapper();

    bool init();

    /**
     * @brief Blocking read operation to read nBytes or till the data transfer
     * is complete
     */
    int read(void *buffer, size_t nBytes);

    /**
     * @brief Blocking write operation
     */
    int write(void *buf, size_t nChars);

    /**
     * @brief Write a string to the serial, comprising the '\0' character
     */
    int writeString(const char *buffer);

private:
    /**
     * @brief Creates and opens the serial port for communication between
     * OBSW and simulation device
     */
    bool serialCommSetup();

    miosix::STM32Serial *serial;  ///< pointer to the serial object
    std::string serialPortName;   ///< Port name of the serial port that has to
                                  ///< be created for the communication
    int fd;  ///< Stores the file descriptor of the serial port file opened for
             ///< trasmission
};

}  // namespace Boardcore