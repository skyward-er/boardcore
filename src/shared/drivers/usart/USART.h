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
#include <interfaces/arch_registers.h>
#include <miosix.h>
#include <utils/ClockUtils.h>

#include <string>

#include "arch/common/drivers/serial.h"

#ifndef USE_MOCK_PERIPHERALS
using USARTType = USART_TypeDef;
#else
// TODO: Create test utils
using USARTType = USART_TypeDef;
#endif

#if defined(UART8)
#define N_USART_PORTS 8
#elif defined(UART7)
#define N_USART_PORTS 7
#elif defined(USART6)
#define N_USART_PORTS 6
#elif defined(UART5)
#define N_USART_PORTS 5
#elif defined(UART4)
#define N_USART_PORTS 4
#elif defined(USART3)
#define N_USART_PORTS 3
#elif defined(USART2)
#define N_USART_PORTS 2
#elif defined(USART1)
#define N_USART_PORTS 1
#else
#error "Your architecture doesn't support UART"
#endif

namespace Boardcore
{

/**
 * @brief Abstract class that implements the interface for the USART/UART serial
 * communication.
 */
class USARTInterface
{
public:
    /**
     * @brief Constructor of the USART in order to assign usart and baudrate.
     * @param usart Pointer to the USART interface.
     * @param baudrate Baudrate in bit per second. Default values are [2400,
     * 9600, 19200, 38400, 57600, 115200, 230400, 256000, 460800, 921600]
     */
    explicit USARTInterface(USARTType *usart, int baudrate);

    virtual ~USARTInterface() = 0;

    /**
     * @brief Blocking read operation to read nBytes or till the data transfer
     * is complete.
     * @param buffer Buffer that will contain the received data.
     * @param nBytes Maximum size of the buffer.
     * @return If operation succeeded.
     */
    [[nodiscard]] virtual bool readBlocking(void *buffer, size_t nBytes)
    {
        size_t temp;
        return readImpl(buffer, nBytes, temp, true);
    };

    /**
     * @brief Blocking read operation to read nBytes or till the data transfer
     * is complete.
     * @param buffer Buffer that will contain the received data.
     * @param nBytes Maximum size of the buffer.
     * @param nBytesRead Number of bytes read in the transaction.
     * @return If operation succeeded.
     */
    [[nodiscard]] virtual bool readBlocking(void *buffer, size_t nBytes,
                                            size_t &nBytesRead)
    {
        return readImpl(buffer, nBytes, nBytesRead, true);
    };

    /**
     * @brief Blocking write operation.
     * @param buffer Buffer that contains the data to be sent.
     * @param nBytes Bytes to be sent.
     */
    virtual void write(const void *buf, size_t nBytes) = 0;

    /**
     * @brief Write a string to the serial, comprising the '\0' character.
     * @param buffer Buffer that contains the string to be sent.
     */
    virtual void writeString(const char *buffer) = 0;

    /**
     * @brief Returns the id of the serial.
     */
    int getId() { return id; };

protected:
    /**
     * @brief Read method implementation that supports both blocking and
     * non-blocking mode and the return of the number of bytes read.
     * @param buffer Buffer that will contain the received data.
     * @param nBytes Maximum size of the buffer.
     * @param nBytesRead Number of bytes read.
     * @param blocking Whether the read should block or not; in case it isn't
     * blocking the read could return also 0 bytes.
     * @return If operation succeeded.
     */
    virtual bool readImpl(void *buffer, size_t nBytes, size_t &nBytesRead,
                          const bool blocking) = 0;

    USARTType *usart;
    int id = -1;                 ///< Can be from 1 to 8, -1 is invalid.
    IRQn_Type irqn;              ///< IRQ number
    std::string serialPortName;  ///< Port name of the port that has to be
                                 ///< created for the communication
    int baudrate;  ///< Baudrate of the serial communication; standard ones
                   ///< are [2400, 9600, 19200, 38400, 57600, 115200,
                   ///< 230400, 256000, 460800, 921600]

    PrintLogger logger = Logging::getLogger("usart");
};

/**
 * @brief Driver for STM32F4 low level USART/UART peripheral.
 *
 * It allows to configure some low level parameters such as word length, parity,
 * stop bits and oversampling.
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
     * @brief Interrupt handler that deals with receive and idle interrupts.
     *
     * Used to implement the reads, fills up a buffer when the interrupt is
     * fired.
     */
    void IRQhandleInterrupt();

    /**
     * @brief Automatically enables the peripheral and timer peripheral clock.
     *
     * Sets the default values for all the parameters (1 stop bit, 8 bit data,
     * no control flow and no oversampling).
     * Requires that the gpios are configured for the choosen uart.
     *
     * @param usart structure that represents the usart peripheral [accepted
     * are: USART1, USART2, USART3, UART4, UART5, USART6, UART7, UART8].
     * @param baudrate Baudrate in bit per second. Default values are [2400,
     * 9600, 19200, 38400, 57600, 115200, 230400, 256000, 460800, 921600]
     */
    USART(USARTType *usart, int baudrate,
          unsigned int queueLen = usart_queue_default_capacity);

    ///< Delete copy/move constructors/operators.
    USART(const USART &)            = delete;
    USART &operator=(const USART &) = delete;
    USART(USART &&)                 = delete;
    USART &operator=(USART &&)      = delete;

    /**
     * @brief Disables the flags for the generation of the interrupts, the IRQ
     * from the NVIC, the peripheral and removes his pointer from the ports
     * list.
     */
    ~USART() override;

    /**
     * @brief Non-blocking read operation to read nBytes or till the data
     * transfer is complete.
     * @warning could also read 0 bytes.
     *
     * @param buffer Buffer that will contain the received data.
     * @param nBytes Maximum size of the buffer.
     * @return If operation succeeded.
     */
    [[nodiscard]] bool read(void *buffer, size_t nBytes)
    {
        size_t temp;
        return readImpl(buffer, nBytes, temp, false);
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
    [[nodiscard]] bool read(void *buffer, size_t nBytes, size_t &nBytesRead)
    {
        return readImpl(buffer, nBytes, nBytesRead, false);
    };

    /**
     * @brief Blocking write operation.
     * @param buffer Buffer that contains the data to be sent.
     * @param nBytes Bytes to be sent.
     */
    void write(const void *buf, size_t nBytes);

    /**
     * @brief Write a string to the serial, comprising the '\0' character.
     * @param buffer Buffer that contains the string to be sent.
     */
    void writeString(const char *buffer);

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
    void clearQueue();

private:
    /**
     * @brief Read method implementation that supports both
     * blocking/non-blocking mode and the return of the number of bytes read.
     * @param buffer Buffer that will contain the received data.
     * @param nBytes Maximum size of the buffer.
     * @param nBytesRead Number of bytes read.
     * @param blocking Whether the read should block or not; in case it isn't
     * blocking the read could return also 0 bytes.
     * @return If operation succeeded.
     */
    [[nodiscard]] bool readImpl(void *buffer, size_t nBytes, size_t &nBytesRead,
                                const bool blocking) override;

    miosix::FastMutex rxMutex;  ///< mutex for receiving on serial
    miosix::FastMutex txMutex;  ///< mutex for transmitting on serial

    ///< Pointer to the waiting on receive thread
    miosix::Thread *rxWaiting = 0;

    miosix::DynUnsyncQueue<char> rxQueue;  ///< Receiving queue
    bool idle             = true;          ///< Receiver idle
    ParityBit parity      = ParityBit::NO_PARITY;
    WordLength wordLength = WordLength::BIT8;
    int stopBits          = 1;      ///< Number of stop bits [1,2]
    bool over8            = false;  ///< Oversalmpling 8 bit
    bool error            = false;  ///< Error occurred while receiving message

    ///< Default queue length
    const static unsigned int usart_queue_default_capacity = 256;
};

/**
 * @brief Wrapper for the STM32Serial driver in miosix.
 */
class STM32SerialWrapper : public USARTInterface
{
public:
    /**
     * @brief Initializes the serialPortName and initializes the default pins,
     * which are:
     * - USART1: tx=PA9  rx=PA10
     * - USART2: tx=PA2  rx=PA3
     * - USART3: tx=PB10 rx=PB11
     * @param usart structure that represents the usart peripheral [accepted
     * are: USART1, USART2, USART3, UART4].
     * @param baudrate baudrate in bit per second. Default values are [2400,
     * 9600, 19200, 38400, 57600, 115200, 230400, 256000, 460800, 921600]
     */
    STM32SerialWrapper(USARTType *usart, int baudrate);

    /**
     * @brief Initializes the serialPortName and initializes the serial port
     * using custom pins.
     * @param usart structure that represents the usart peripheral [accepted
     * are: USART1, USART2, USART3, UART4].
     * @param baudrate baudrate in bit per second. Default values are [2400,
     * 9600, 19200, 38400, 57600, 115200, 230400, 256000, 460800, 921600]
     * @param tx Tranmission pin
     * @param rx Reception pin
     */
    STM32SerialWrapper(USARTType *usart, int baudrate, miosix::GpioPin tx,
                       miosix::GpioPin rx);

    ///< Delete copy/move constructors/operators.
    STM32SerialWrapper(const STM32SerialWrapper &)            = delete;
    STM32SerialWrapper &operator=(const STM32SerialWrapper &) = delete;
    STM32SerialWrapper(STM32SerialWrapper &&)                 = delete;
    STM32SerialWrapper &operator=(STM32SerialWrapper &&)      = delete;

    /**
     * @brief Removes the device from the list of the devices and closes the
     * file of the device.
     */
    ~STM32SerialWrapper();

    /**
     * @brief Blocking write operation.
     * @param buffer Buffer that contains the data to be sent.
     * @param nBytes Bytes to be sent.
     */
    void write(const void *buf, size_t nBytes);

    /**
     * @brief Write a string to the serial, comprising the '\0' character.
     * @param buffer Buffer that contains the string to be sent.
     */
    void writeString(const char *buffer);

private:
    /**
     * @brief Read method implementation that supports both
     * blocking/non-blocking mode and the return of the number of bytes read.
     * @param buffer Buffer that will contain the received data.
     * @param nBytes Maximum size of the buffer.
     * @param nBytesRead Number of bytes read.
     * @param blocking Whether the read should block or not; in case it isn't
     * blocking the read could return also 0 bytes.
     * @return If operation succeeded.
     */
    [[nodiscard]] bool readImpl(void *buffer, size_t nBytes, size_t &nBytesRead,
                                const bool blocking) override;

    /**
     * @brief Creates a device that represents the serial port, adds it to the
     * file system and opens the file that represents the device.
     */
    bool serialCommSetup();

    miosix::STM32Serial *serial;  ///< Pointer to the serial object

    ///< File descriptor of the serial port file opened for transmission
    int fd;
};

}  // namespace Boardcore
