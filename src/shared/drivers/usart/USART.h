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
// TODO: Create test utils
#endif

#ifdef STM32F429xx
#define N_USART_PORTS 8
#else
#define N_USART_PORTS 6
#endif

// A nice feature of the stm32 is that the USART are connected to the same
// GPIOS in all families, stm32f1, f2, f4 and l1. Additionally, USART1 and
// USART6 are always connected to the APB2, while the other USART/UARTs are
// connected to the APB1.

// USART1: AF7
typedef miosix::Gpio<GPIOB_BASE, 6> u1tx1;
typedef miosix::Gpio<GPIOB_BASE, 7> u1rx1;
typedef miosix::Gpio<GPIOA_BASE, 9> u1tx2;
typedef miosix::Gpio<GPIOA_BASE, 10> u1rx2;
// typedef miosix::Gpio<GPIOA_BASE, 11> u1cts;
// typedef miosix::Gpio<GPIOA_BASE, 12> u1rts;

// USART2: AF7
typedef miosix::Gpio<GPIOA_BASE, 2> u2tx1;
typedef miosix::Gpio<GPIOA_BASE, 3> u2rx1;
typedef miosix::Gpio<GPIOD_BASE, 5> u2tx2;
typedef miosix::Gpio<GPIOD_BASE, 6> u2rx2;
// typedef miosix::Gpio<GPIOA_BASE, 0> u2cts;
// typedef miosix::Gpio<GPIOA_BASE, 1> u2rts;

// USART3: AF7
typedef miosix::Gpio<GPIOB_BASE, 10> u3tx1;
typedef miosix::Gpio<GPIOB_BASE, 11> u3rx1;
typedef miosix::Gpio<GPIOD_BASE, 8> u3tx2;
typedef miosix::Gpio<GPIOD_BASE, 9> u3rx2;
// typedef miosix::Gpio<GPIOB_BASE, 13> u3cts;
// typedef miosix::Gpio<GPIOB_BASE, 14> u3rts;

// UART4: AF8
typedef miosix::Gpio<GPIOA_BASE, 0> u4tx1;
typedef miosix::Gpio<GPIOA_BASE, 1> u4rx1;
typedef miosix::Gpio<GPIOC_BASE, 10> u4tx2;
typedef miosix::Gpio<GPIOC_BASE, 11> u4rx2;

// UART5: AF8
typedef miosix::Gpio<GPIOC_BASE, 12> u5tx;
typedef miosix::Gpio<GPIOD_BASE, 2> u5rx;

// USART6: AF8
typedef miosix::Gpio<GPIOC_BASE, 6> u6tx1;
typedef miosix::Gpio<GPIOC_BASE, 7> u6rx1;
#ifdef STM32F429xx
typedef miosix::Gpio<GPIOG_BASE, 14> u6tx2;
typedef miosix::Gpio<GPIOG_BASE, 9> u6rx2;

// USART7: AF8
typedef miosix::Gpio<GPIOE_BASE, 8> u7tx1;
typedef miosix::Gpio<GPIOE_BASE, 7> u7rx1;
typedef miosix::Gpio<GPIOF_BASE, 7> u7tx2;
typedef miosix::Gpio<GPIOF_BASE, 6> u7rx2;

// USART8: AF8
typedef miosix::Gpio<GPIOE_BASE, 1> u8tx;
typedef miosix::Gpio<GPIOE_BASE, 0> u8rx;
#endif  // STM32F429xx

namespace Boardcore
{
/**
 * @brief Abstract class that implements the interface for the USART/UART serial
 * communication.
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

    virtual ~USARTInterface() = 0;

    /**
     * @brief Initializes the peripheral enabling his interrupts, the interrupts
     * in the NVIC.
     *
     * All the setup phase (with the setting of the pins and their alternate
     * functions) must be done before the initialization of the peripheral.
     */
    virtual bool init() = 0;

    /**
     * @brief Blocking read operation to read nBytes or till the data transfer
     * is complete.
     */
    virtual int read(void *buffer, size_t nBytes) = 0;

    /**
     * @brief Blocking write operation.
     */
    virtual int write(void *buf, size_t nChars) = 0;

    /**
     * @brief Write a string to the serial, comprising the '\0' character.
     */
    virtual int writeString(const char *buffer) = 0;

    /**
     * @brief Returns the id of the serial.
     */
    int getId() { return id; };

protected:
    /**
     * @brief Initializes the pins with the appropriate alternate functions.
     *
     * @param tx Tranmission pin.
     * @param nAFtx Tranmission pin alternate function.
     * @param rx Reception pin.
     * @param nAFrx Reception pin alternate function.
     */
    bool initPins(miosix::GpioPin tx, int nAFtx, miosix::GpioPin rx, int nAFrx);

    ///< True if initPins() already called successfully, false otherwise
    bool pinInitialized = false;

    miosix::GpioPin tx{GPIOA_BASE, 0};
    miosix::GpioPin rx{GPIOA_BASE, 0};

    USARTType *usart;
    int id           = 1;  ///< Can be 1, 2, 3, 4, 5, 6, 7, 8
    bool initialized = false;
    Baudrate baudrate;  ///< Baudrate of the serial communication
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

    ///< Pointer to serial port classes to let interrupts access the classes
    static USART *ports[];

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
     * Initializes the serial port using the default pins, which are:
     * - USART1: tx=PA9  rx=PA10
     * - USART2: tx=PA2  rx=PA3
     * - USART3: tx=PB10 rx=PB11
     * - UART4:  tx=PA0  rx=PA1
     * - UART5:  tx=PC12 rx=PD2
     * - USART6: tx=PC6  rx=PC7
     * - UART7:  tx=PE8  rx=PE7
     * - UART8:  tx=PE1  rx=PE0
     *
     * @param usart structure that represents the usart peripheral [accepted
     * are: USART1, USART2, USART3, UART4, UART5, USART6, UART7, UART8].
     * @param baudrate member of the enum Baudrate that represents the baudrate
     * with which the communication will take place.
     */
    USART(USARTType *usart, Baudrate baudrate,
          unsigned int queueLen = usart_queue_default_capacity);

    /**
     * @brief Automatically enables the peripheral and timer peripheral clock.
     *
     * Sets the default values for all the parameters (1 stop bit, 8 bit data,
     * no control flow and no oversampling).
     * Initializes the serial port using custom pins.
     *
     * @param usart Structure that represents the usart peripheral [accepted
     * are: USART1, USART2, USART3, UART4, UART5, USART6, UART7, UART8].
     * @param baudrate Member of the enum Baudrate that represents the baudrate
     * with which the communication will take place.
     * @param tx Tranmission pin.
     * @param rx Reception pin.
     */
    USART(USARTType *usart, Baudrate baudrate, miosix::GpioPin tx,
          miosix::GpioPin rx,
          unsigned int queueLen = usart_queue_default_capacity);

    /**
     * @brief Disables the flags for the generation of the interrupts, the IRQ
     * from the NVIC, the peripheral and removes his pointer from the ports
     * list.
     */
    ~USART();

    /**
     * @brief Initializes the peripheral enabling his interrupts, the interrupts
     * in the NVIC and setting the pins with the appropriate alternate
     * functions.
     *
     * All the setup phase must be done before the initialization of the
     * peripheral. The pins must be initialized before calling this function.
     */
    bool init();

    /**
     * @brief Blocking read operation to read nBytes or till the data transfer
     * is complete.
     */
    int read(void *buffer, size_t nBytes);

    /**
     * @brief Blocking write operation.
     */
    int write(void *buf, size_t nChars);

    /**
     * @brief Write a string to the serial, comprising the '\0' character.
     */
    int writeString(const char *buffer);

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
     * @param pb Baudrate element that represents the baudrate.
     */
    void setBaudrate(Baudrate br);

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
    void commonConstructor(USARTType *usart, Baudrate baudrate);

    IRQn_Type irqn;
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
     * are: USART1, USART2, USART3].
     * @param baudrate member of the enum Baudrate that represents the baudrate
     * with which the communication will take place.
     */
    STM32SerialWrapper(USARTType *usart, Baudrate baudrate);

    /**
     * @brief Initializes the serialPortName and initializes the serial port
     * using custom pins.
     * @param usart structure that represents the usart peripheral [accepted
     * are: USART1, USART2, USART3].
     * @param baudrate member of the enum Baudrate that represents the baudrate
     * with which the communication will take place.
     * @param tx Tranmission pin
     * @param rx Reception pin
     */
    STM32SerialWrapper(USARTType *usart, Baudrate baudrate, miosix::GpioPin tx,
                       miosix::GpioPin rx);

    /**
     * @brief Removes the device from the list of the devices and closes the
     * file of the device.
     */
    ~STM32SerialWrapper();

    /**
     * @brief Initializes the peripheral.
     *
     * @see{STM32SerialWrapper::serialCommSetup}
     */
    bool init();

    /**
     * @brief Blocking read operation to read nBytes or till the data transfer
     * is complete.
     */
    int read(void *buffer, size_t nBytes);

    /**
     * @brief Blocking write operation.
     */
    int write(void *buf, size_t nChars);

    /**
     * @brief Write a string to the serial, comprising the '\0' character.
     */
    int writeString(const char *buffer);

private:
    /**
     * @brief Creates a device that represents the serial port, adds it to the
     * file system and opens the file that represents the device.
     */
    bool serialCommSetup();

    miosix::STM32Serial *serial;  ///< Pointer to the serial object

    //< Port name of the port that has to be created for the communication
    std::string serialPortName;

    ///< File descriptor of the serial port file opened for transmission
    int fd;
};

}  // namespace Boardcore
