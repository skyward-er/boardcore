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

#include "drivers/usart/USART.h"

#include <fcntl.h>
#include <stdio.h>
#include <utils/Debug.h>

#include <string>

#include "arch/common/drivers/serial.h"
#include "filesystem/file_access.h"
#include "miosix.h"

///< Pointer to serial port classes to let interrupts access the classes
Boardcore::USART *ports[N_USART_PORTS];

#ifdef USART1
/**
 * \internal interrupt routine for usart1 miosix implementation
 */
void usart1irqImpl();

/**
 * \internal Interrupt routine for usart1 actual implementation.
 */
void __attribute__((used)) usart1irqImplBoardcore()
{
    Boardcore::USART *port_boardcore = ports[0];
    if (port_boardcore)
    {
        port_boardcore->IRQhandleInterrupt();
    }
    else
    {
        usart1irqImpl();
    }
}

/**
 * \internal Interrupt routine for usart1.
 */
void __attribute__((naked, used)) USART1_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z22usart1irqImplBoardcorev");
    restoreContext();
}
#endif

#ifdef USART2
/**
 * \internal interrupt routine for usart2 miosix implementation
 */
void usart2irqImpl();

/**
 * \internal Interrupt routine for usart2 actual implementation.
 */
void __attribute__((used)) usart2irqImplBoardcore()
{
    Boardcore::USART *port_boardcore = ports[1];
    if (port_boardcore)
    {
        port_boardcore->IRQhandleInterrupt();
    }
    else
    {
        usart2irqImpl();
    }
}

/**
 * \internal Interrupt routine for usart2.
 */
void __attribute__((naked, used)) USART2_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z22usart2irqImplBoardcorev");
    restoreContext();
}
#endif

#ifdef USART3
/**
 * \internal interrupt routine for usart3 miosix implementation
 */
void usart3irqImpl();

/**
 * \internal Interrupt routine for usart3 actual implementation.
 */
void __attribute__((used)) usart3irqImplBoardcore()
{
    Boardcore::USART *port_boardcore = ports[2];
    if (port_boardcore)
    {
        port_boardcore->IRQhandleInterrupt();
    }
    else
    {
        usart3irqImpl();
    }
}

/**
 * \internal Interrupt routine for usart3.
 */
void __attribute__((naked, used)) USART3_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z22usart3irqImplBoardcorev");
    restoreContext();
}
#endif

#ifdef UART4
/**
 * \internal Interrupt routine for uart4 actual implementation.
 */
void __attribute__((used)) uart4irqImplBoardcore()
{
    Boardcore::USART *port_boardcore = ports[3];
    if (port_boardcore)
    {
        port_boardcore->IRQhandleInterrupt();
    }
}

/**
 * \internal Interrupt routine for uart4.
 */
void __attribute__((naked, used)) UART4_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z21uart4irqImplBoardcorev");
    restoreContext();
}
#endif

#ifdef UART5
/**
 * \internal Interrupt routine for uart5 actual implementation.
 */
void __attribute__((used)) uart5irqImplBoardcore()
{
    Boardcore::USART *port_boardcore = ports[4];
    if (port_boardcore)
    {
        port_boardcore->IRQhandleInterrupt();
    }
}

/**
 * \internal Interrupt routine for uart5.
 */
void __attribute__((naked, used)) UART5_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z21uart5irqImplBoardcorev");
    restoreContext();
}
#endif

#ifdef USART6
/**
 * \internal Interrupt routine for usart6 actual implementation.
 */
void __attribute__((used)) usart6irqImplBoardcore()
{
    Boardcore::USART *port_boardcore = ports[5];
    if (port_boardcore)
    {
        port_boardcore->IRQhandleInterrupt();
    }
}

/**
 * \internal Interrupt routine for usart6.
 */
void __attribute__((naked, used)) USART6_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z22usart6irqImplBoardcorev");
    restoreContext();
}
#endif

#ifdef UART7
/**
 * \internal Interrupt routine for uart7 actual implementation.
 */
void __attribute__((used)) uart7irqImplBoardcore()
{
    Boardcore::USART *port_boardcore = ports[6];
    if (port_boardcore)
    {
        port_boardcore->IRQhandleInterrupt();
    }
}

/**
 * \internal Interrupt routine for uart7.
 */
void __attribute__((naked, used)) UART7_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z21uart7irqImplBoardcorev");
    restoreContext();
}
#endif

#ifdef UART8
/**
 * \internal Interrupt routine for uart8 actual implementation.
 */
void __attribute__((used)) uart8irqImplBoardcore()
{
    Boardcore::USART *port_boardcore = ports[7];
    if (port_boardcore)
    {
        port_boardcore->IRQhandleInterrupt();
    }
}

/**
 * \internal Interrupt routine for uart8.
 */
void __attribute__((naked, used)) UART8_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z21uart8irqImplBoardcorev");
    restoreContext();
}
#endif

namespace Boardcore
{

USARTInterface::USARTInterface(USARTType *usart, int baudrate)
    : usart(usart), baudrate(baudrate)
{
    // Setting the id of the serial port
    switch (reinterpret_cast<uint32_t>(usart))
    {
#ifdef USART1
        case USART1_BASE:
            this->id             = 1;
            irqn                 = USART1_IRQn;
            this->serialPortName = std::string("usart1");
            break;
#endif
#ifdef USART2
        case USART2_BASE:
            this->id             = 2;
            irqn                 = USART2_IRQn;
            this->serialPortName = std::string("usart2");
            break;
#endif
#ifdef USART3
        case USART3_BASE:
            this->id             = 3;
            irqn                 = USART3_IRQn;
            this->serialPortName = std::string("usart3");
            break;
#endif
#ifdef UART4
        case UART4_BASE:
            this->id             = 4;
            irqn                 = UART4_IRQn;
            this->serialPortName = std::string("uart4");
            break;
#endif
#ifdef UART5
        case UART5_BASE:
            this->id             = 5;
            irqn                 = UART5_IRQn;
            this->serialPortName = std::string("uart5");
            break;
#endif
#ifdef USART6
        case USART6_BASE:
            this->id             = 6;
            irqn                 = USART6_IRQn;
            this->serialPortName = std::string("usart6");
            break;
#endif
#ifdef UART7
        case UART7_BASE:
            this->id             = 7;
            irqn                 = UART7_IRQn;
            this->serialPortName = std::string("uart7");
            break;
#endif
#ifdef UART8
        case UART8_BASE:
            this->id             = 8;
            irqn                 = UART8_IRQn;
            this->serialPortName = std::string("uart8");
            break;
#endif
        default:
            LOG_ERR(logger, "USART selected not supported!");
            D(assert(false && "USART selected not supported!"));
    }
}

USARTInterface::~USARTInterface() {}

void USART::IRQhandleInterrupt()
{
    char c;
    bool received = false;
    bool framingError;

#ifndef _ARCH_CORTEXM7_STM32F7
    // If read data register is empty then read data
    received = ((usart->SR & USART_SR_RXNE) == 0 ? false : true);
    // If no error put data in buffer
    framingError = ((usart->SR & USART_SR_FE) == 0 ? false : true);
    idle         = ((usart->SR & USART_SR_IDLE) == 0 ? false : true);
    // Always read data, since this clears interrupt flags
    c = usart->DR;
#else
    // If read data register is empty then read data
    received = ((usart->ISR & USART_ISR_RXNE) == 0 ? false : true);
    // If no error put data in buffer
    framingError = ((usart->ISR & USART_ISR_FE) == 0 ? false : true);
    idle         = ((usart->ISR & USART_ISR_IDLE) == 0 ? false : true);
    // Clears interrupt flags
    usart->ICR = USART_ICR_IDLECF;
    // Always read data, since this clears interrupt flags
    c = usart->RDR;
#endif

    // If we received some data without framing error but the tryPut failed,
    // report a FIFO overflow
    if (framingError || (received && !rxQueue.tryPut(c)))
    {
        error = true;
    }

    // Wake up thread if communication finished (idle state), buffer reached
    // half of his capacity or error occurred
    if (error || idle || (rxQueue.size() >= rxQueue.capacity() / 2))
    {
        // Enough data in buffer or idle line, awake thread
        if (rxWaiting)
        {
            rxWaiting->IRQwakeup();
            rxWaiting = nullptr;
        }
    }
}

USART::USART(USARTType *usart, int baudrate, unsigned int queueLen)
    : USARTInterface(usart, baudrate), rxQueue(queueLen)
{
    // Enabling the peripheral on the right APB
    ClockUtils::enablePeripheralClock(usart);

    // Setting the baudrate chosen
    setBaudrate(baudrate);

    // Default settings
    setStopBits(1);
    setWordLength(USART::WordLength::BIT8);
    setParity(USART::ParityBit::NO_PARITY);
    setOversampling(false);

    {
        miosix::FastInterruptDisableLock dLock;

        // Enable usart, receiver, receiver interrupt and idle interrupt
        usart->CR1 |= USART_CR1_UE        // Enabling the uart peripheral
                      | USART_CR1_RXNEIE  // Interrupt on data received
                      | USART_CR1_IDLEIE  // interrupt on idle line
                      | USART_CR1_TE      // Transmission enabled
                      | USART_CR1_RE;     // Reception enabled

        // Sample only one bit
        usart->CR3 |= USART_CR3_ONEBIT;
    }

    // Add to the array of usarts so that the interrupts can see it
    ports[id - 1] = this;

    // Enabling the interrupt for the relative serial port
    NVIC_SetPriority(irqn, 15);
    NVIC_EnableIRQ(irqn);

    // Clearing the queue for random data read at the beginning
    this->clearQueue();
}

USART::~USART()
{
    miosix::FastInterruptDisableLock dLock;

    // Take out the usart object we are going to destruct
    ports[this->id - 1] = nullptr;

    // Disabling the usart
    usart->CR1 &= ~(USART_CR1_UE | USART_CR1_TE | USART_CR1_RE);

    // Disabling the interrupt of the serial port
    NVIC_DisableIRQ(irqn);
}

void USART::setWordLength(WordLength wordLength)
{
    miosix::FastInterruptDisableLock dLock;
    (wordLength == WordLength::BIT8 ? usart->CR1 &= ~USART_CR1_M
                                    : usart->CR1 |= USART_CR1_M);
    this->wordLength = wordLength;
}

void USART::setParity(ParityBit parity)
{
    miosix::FastInterruptDisableLock dLock;
    (parity == ParityBit::NO_PARITY ? usart->CR1 &= ~USART_CR1_PCE
                                    : usart->CR1 |= USART_CR1_PCE);
    this->parity = parity;
}

void USART::setStopBits(int stopBits)
{
    miosix::FastInterruptDisableLock dLock;
    this->stopBits = stopBits;
    usart->CR2 &= ~USART_CR2_STOP;
    if (stopBits == 2)
    {
        usart->CR2 |= USART_CR2_STOP_1;
    }
}

void USART::setOversampling(bool oversampling)
{
    miosix::FastInterruptDisableLock dLock;
    this->over8 = oversampling;
    (oversampling ? usart->CR1 |= USART_CR1_OVER8
                  : usart->CR1 &= ~USART_CR1_OVER8);
}

void USART::setBaudrate(int baudrate)
{
    /*
     * Baudrate setting:
     * fixed point: mantissa first 12 bits, other 4 bits are the fixed point
     * value of the fraction.
     * - if over8==0 => DIV_Fraction[3:0] (all fraction used)
     * USART_DIV = f/(16*baud)
     * - if over8==1 => 0+DIV_Fraction[2:0] (first bit of fraction is 0)
     * USART_DIV = f/(8*baud)
     */
    miosix::InterruptDisableLock dLock;

    // USART1 and USART6 are always connected to the APB2, while all the others
    // UART/USART peripherals are always connected to APB1
    uint32_t f = ClockUtils::getAPBPeripheralsClock(
        (id == 1 || id == 6 ? ClockUtils::APB::APB2     // High speed APB2
                            : ClockUtils::APB::APB1));  // Low speed APB1

    // <<4 in order to shift to left of 4 positions, to create a fixed point
    // number of 4 decimal digits /8 == >>3 in order to divide per 8 (from the
    // formula in the datasheet). So it should be << 1 but, in order to make the
    // approximation later, we shift it of 2 positions
    uint32_t brr = ((f << 2) / (((int)baudrate * (over8 ? 1 : 2))));

    // rounding to the nearest
    usart->BRR = (brr / 2) + (brr & 1);

    this->baudrate = baudrate;
}

bool USART::readImpl(void *buffer, size_t nBytes, size_t &nBytesRead,
                     const bool blocking)
{
    miosix::Lock<miosix::FastMutex> l(rxMutex);

    char *buf     = reinterpret_cast<char *>(buffer);
    size_t result = 0;
    error         = false;
    miosix::FastInterruptDisableLock dLock;
    for (;;)
    {
        // Try to get all the data possible from the queue
        for (; result < nBytes; result++)
        {
            if (!rxQueue.tryGet(buf[result]))
                break;

            // This is here just not to keep IRQ disabled for the whole loop
            miosix::FastInterruptEnableLock eLock(dLock);
        }

        // If blocking, we are waiting for at least one byte of data before
        // returning. If not blocking, in the case the bus is idle we return
        // anyway
        if ((result == nBytes) || (idle && (!blocking || (result > 0))))
            break;

        // Wait for data in the queue
        do
        {
            rxWaiting = miosix::Thread::IRQgetCurrentThread();
            miosix::Thread::IRQwait();
            {
                miosix::FastInterruptEnableLock eLock(dLock);
                miosix::Thread::yield();
            }
        } while (rxWaiting);
    }
    nBytesRead = result;

    return (result > 0);
}

void USART::write(const void *buffer, size_t nBytes)
{
    miosix::Lock<miosix::FastMutex> l(txMutex);

    // TODO: Use the send complete interrupt in order not to have a busy while
    // loop waiting
    const char *buf = reinterpret_cast<const char *>(buffer);
    size_t i;
    for (i = 0; i < nBytes; i++)
    {
#ifndef _ARCH_CORTEXM7_STM32F7
        while ((usart->SR & USART_SR_TXE) == 0)
            ;
        usart->DR = *buf++;
#else
        while ((usart->ISR & USART_ISR_TXE) == 0)
            ;
        usart->TDR = *buf++;
#endif
    }
}

void USART::writeString(const char *buffer)
{
    int i = 0;
    miosix::Lock<miosix::FastMutex> l(txMutex);

    // Send everything, also the ending '\0' character
#ifndef _ARCH_CORTEXM7_STM32F7
    usart->DR = *buffer;
#else
    usart->TDR = *buffer;
#endif

    i++;

    while (*buffer != '\0')
    {
        buffer++;

#ifndef _ARCH_CORTEXM7_STM32F7
        while (!(usart->SR & USART_SR_TXE))
            ;
        usart->DR = *buffer;
#else
        while ((usart->ISR & USART_ISR_TXE) == 0)
            ;
        usart->TDR = *buffer;
#endif

        i++;
    };
}

void USART::clearQueue()
{
    char buf[usart_queue_default_capacity];
    rxQueue.reset();
    while (read(buf, usart_queue_default_capacity))
        ;
    rxQueue.reset();
}

STM32SerialWrapper::STM32SerialWrapper(USARTType *usart, int baudrate)
    : USARTInterface(usart, baudrate)
{
    if (this->id < 1 || this->id > 4)
    {
        LOG_ERR(logger, "USART selected not supported for STM32SerialWrapper!");
        D(assert(false &&
                 "USART selected not supported for STM32SerialWrapper!"));
    }

    // Creates and adds the serial port to the devices
    this->serial = new miosix::STM32Serial(id, baudrate);

    if (!serialCommSetup())
    {
        LOG_ERR(logger,
                "[STM32SerialWrapper] can't initialize serial communication!");
        D(assert(false &&
                 "[STM32SerialWrapper] Error : can't initialize serial "
                 "communication!\n"));
    }
}

STM32SerialWrapper::STM32SerialWrapper(USARTType *usart, int baudrate,
                                       miosix::GpioPin tx, miosix::GpioPin rx)
    : USARTInterface(usart, baudrate)
{
    if (this->id < 1 || this->id > 4)
    {
        LOG_ERR(logger, "USART selected not supported for STM32SerialWrapper!");
        D(assert(false &&
                 "USART selected not supported for STM32SerialWrapper!"));
    }

    // Creates and adds the serial port to the devices
    this->serial = new miosix::STM32Serial(id, baudrate, tx, rx);

    if (!serialCommSetup())
    {
        LOG_ERR(logger,
                "[STM32SerialWrapper] can't initialize serial communication!");
        D(assert(false &&
                 "[STM32SerialWrapper] Error : can't initialize serial "
                 "communication!\n"));
    }
}

STM32SerialWrapper::~STM32SerialWrapper()
{
    miosix::intrusive_ref_ptr<miosix::DevFs> devFs =
        miosix::FilesystemManager::instance().getDevFs();
    close(fd);
    devFs->remove(serialPortName.c_str());
}

bool STM32SerialWrapper::serialCommSetup()
{
    // Adds a device to the file system
    if (!miosix::FilesystemManager::instance().getDevFs()->addDevice(
            serialPortName.c_str(),
            miosix::intrusive_ref_ptr<miosix::Device>(serial)))
    {
        return false;
    }

    // Path string "/dev/<name_of_port>" for the port we want to open
    std::string serialPortPath = "/dev/" + serialPortName;

    // Open serial port
    fd = open(serialPortPath.c_str(), O_RDWR);

    if (fd <= -1)
    {
        TRACE("Cannot open %s\n", serialPortPath.c_str());
        return false;
    }

    return true;
}

bool STM32SerialWrapper::readImpl(void *buffer, size_t nBytes,
                                  size_t &nBytesRead, const bool blocking)
{
    // non-blocking read not supported in STM32SerialWrapper
    if (!blocking)
    {
        LOG_ERR(logger,
                "STM32SerialWrapper::read doesn't support non-blocking read");
        D(assert(false &&
                 "STM32SerialWrapper::read doesn't support non-blocking read"));
    }

    size_t n   = ::read(fd, buffer, nBytes);
    nBytesRead = n;

    return (n > 0);
}

void STM32SerialWrapper::write(const void *buffer, size_t nBytes)
{
    ::write(fd, buffer, nBytes);
}

void STM32SerialWrapper::writeString(const char *buffer)
{
    // strlen + 1 in order to send the '/0' terminated string
    ::write(fd, buffer, strlen(buffer) + 1);
}

}  // namespace Boardcore
