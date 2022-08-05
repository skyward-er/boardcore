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

Boardcore::USART *Boardcore::USART::ports[N_USART_PORTS];

/**
 * \internal Interrupt routine for usart1 actual implementation.
 */
void __attribute__((used)) usart1irqImplBoardcore()
{
    Boardcore::USART *port_boardcore = Boardcore::USART::ports[0];
    if (port_boardcore)
        port_boardcore->IRQhandleInterrupt();
    else
    {
        miosix::STM32Serial *port = miosix::STM32Serial::ports[0];
        if (port)
            port->IRQhandleInterrupt();
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

/**
 * \internal Interrupt routine for usart2 actual implementation.
 */
void __attribute__((used)) usart2irqImplBoardcore()
{
    Boardcore::USART *port_boardcore = Boardcore::USART::ports[1];
    if (port_boardcore)
        port_boardcore->IRQhandleInterrupt();
    else
    {
        miosix::STM32Serial *port = miosix::STM32Serial::ports[1];
        if (port)
            port->IRQhandleInterrupt();
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

/**
 * \internal Interrupt routine for usart3 actual implementation.
 */
void __attribute__((used)) usart3irqImplBoardcore()
{
    Boardcore::USART *port_boardcore = Boardcore::USART::ports[2];
    if (port_boardcore)
        port_boardcore->IRQhandleInterrupt();
    else
    {
        miosix::STM32Serial *port = miosix::STM32Serial::ports[2];
        if (port)
            port->IRQhandleInterrupt();
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

/**
 * \internal Interrupt routine for uart4 actual implementation.
 */
void __attribute__((used)) uart4irqImplBoardcore()
{
    Boardcore::USART *port_boardcore = Boardcore::USART::ports[3];
    if (port_boardcore)
        port_boardcore->IRQhandleInterrupt();
    else
    {
        miosix::STM32Serial *port = miosix::STM32Serial::ports[3];
        if (port)
            port->IRQhandleInterrupt();
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

/**
 * \internal Interrupt routine for uart5 actual implementation.
 */
void __attribute__((used)) uart5irqImplBoardcore()
{
    Boardcore::USART *port_boardcore = Boardcore::USART::ports[4];
    if (port_boardcore)
        port_boardcore->IRQhandleInterrupt();
    else
    {
        miosix::STM32Serial *port = miosix::STM32Serial::ports[4];
        if (port)
            port->IRQhandleInterrupt();
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

/**
 * \internal Interrupt routine for usart6 actual implementation.
 */
void __attribute__((used)) usart6irqImplBoardcore()
{
    Boardcore::USART *port_boardcore = Boardcore::USART::ports[5];
    if (port_boardcore)
        port_boardcore->IRQhandleInterrupt();
    else
    {
        miosix::STM32Serial *port = miosix::STM32Serial::ports[5];
        if (port)
            port->IRQhandleInterrupt();
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

#ifdef STM32F429xx

/**
 * \internal Interrupt routine for uart7 actual implementation.
 */
void __attribute__((used)) uart7irqImplBoardcore()
{
    Boardcore::USART *port_boardcore = Boardcore::USART::ports[6];
    if (port_boardcore)
        port_boardcore->IRQhandleInterrupt();
    else
    {
        miosix::STM32Serial *port = miosix::STM32Serial::ports[6];
        if (port)
            port->IRQhandleInterrupt();
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

/**
 * \internal Interrupt routine for uart8 actual implementation.
 */
void __attribute__((used)) uart8irqImplBoardcore()
{
    Boardcore::USART *port_boardcore = Boardcore::USART::ports[7];
    if (port_boardcore)
        port_boardcore->IRQhandleInterrupt();
    else
    {
        miosix::STM32Serial *port = miosix::STM32Serial::ports[7];
        if (port)
            port->IRQhandleInterrupt();
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

#endif  // STM32F429xx

namespace Boardcore
{

USARTInterface::~USARTInterface() {}

void USART::IRQhandleInterrupt()
{
    char c;

    // If read data register is empty then read data
    if (usart->SR & USART_SR_RXNE)
    {
        // Always read data, since this clears interrupt flags
        c = usart->DR;
        // If no error put data in buffer
        if (!(usart->SR & USART_SR_FE))
            if (rxQueue.tryPut(c) == false)  // FIFO overflow
                ;

        idle = false;
    }

    if (usart->SR & USART_SR_IDLE)
        idle = true;

    if (usart->SR & USART_SR_IDLE || rxQueue.size() >= rxQueue.capacity() / 2)
    {
        c = usart->DR;  // Clears interrupt flags

        // Enough data in buffer or idle line, awake thread
        if (rxWaiting)
        {
            rxWaiting->IRQwakeup();
            rxWaiting = 0;
        }
    }
}

USART::USART(USARTType *usart, Baudrate baudrate, unsigned int queueLen)
    : rxQueue(queueLen)
{
    // Setting the id of the serial port
    switch (reinterpret_cast<uint32_t>(usart))
    {
        case USART1_BASE:
            this->id = 1;
            irqn     = USART1_IRQn;
            break;
        case USART2_BASE:
            this->id = 2;
            irqn     = USART2_IRQn;
            break;
        case USART3_BASE:
            this->id = 3;
            irqn     = USART3_IRQn;
            break;
        case UART4_BASE:
            this->id = 4;
            irqn     = UART4_IRQn;
            break;
        case UART5_BASE:
            this->id = 5;
            irqn     = UART5_IRQn;
            break;
        case USART6_BASE:
            this->id = 6;
            irqn     = USART6_IRQn;
            break;
#ifdef STM32F429xx
        case UART7_BASE:
            this->id = 7;
            irqn     = UART7_IRQn;
            break;
        case UART8_BASE:
            this->id = 8;
            irqn     = UART8_IRQn;
            break;
#endif  // STM32F429xx
    }

    this->usart = usart;

    // Enabling the peripheral on the right APB
    ClockUtils::enablePeripheralClock(usart);

    // Enabling the usart peripheral
    {
        miosix::FastInterruptDisableLock dLock;
        usart->CR1 |= USART_CR1_UE;
    }

    // Setting the baudrate chosen
    setBaudrate(baudrate);

    // Default settings
    setStopBits(1);
    setWordLength(USART::WordLength::BIT8);
    setParity(USART::ParityBit::NO_PARITY);
    setOversampling(false);
}

USART::~USART()
{
    miosix::FastInterruptDisableLock dLock;

    // Take out the usart object we are going to destruct
    USART::ports[this->id - 1] = nullptr;

    // Disabling the usart
    usart->CR1 &= ~(USART_CR1_UE | USART_CR1_TE | USART_CR1_RE);

    // Disabling the interrupt of the serial port
    NVIC_DisableIRQ(irqn);
}

bool USART::init()
{
    if (id < 1 || id > MAX_SERIAL_PORTS)
    {
        TRACE("Not supported USART id\n");
        return false;
    }

    {
        miosix::FastInterruptDisableLock dLock;

        // Enable usart, receiver, receiver interrupt and idle interrupt
        usart->CR1 |= USART_CR1_RXNEIE    // Interrupt on data received
                      | USART_CR1_IDLEIE  // interrupt on idle line
                      | USART_CR1_TE      // Transmission enabled
                      | USART_CR1_RE;     // Reception enabled

        // Sample only one bit
        usart->CR3 |= USART_CR3_ONEBIT;

        // Enabling the interrupt for the relative serial port
        NVIC_SetPriority(irqn, 15);
        NVIC_EnableIRQ(irqn);

        // Add to the array of usarts so that the interrupts can see it
        USART::ports[id - 1] = this;
    }

    // Clearing the queue for random data read at the beginning
    miosix::Thread::sleep(1);
    this->clearQueue();

    return true;
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

void USART::setBaudrate(Baudrate baudrate)
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

    // USART1 and USART6 is always connected to the APB2, while all the others
    // UART/USART peripherals are always connected to APB1
    uint32_t f = ClockUtils::getAPBFrequency(
        (id == 1 || id == 6 ? ClockUtils::APB::APB2     // High speed APB2
                            : ClockUtils::APB::APB1));  // Low speed APB1,

    // <<4 in order to shift to left of 4 positions, to create a fixed point
    // number of 4 decimal digits /8 == >>3 in order to divide per 8 (from the
    // formula in the datasheet)
    uint32_t USART_DIV = ((f << 1) / (((int)baudrate * (over8 ? 1 : 2))));

    // rounding to the nearest
    uint32_t brr = (USART_DIV / 2) + (USART_DIV & 1);

    if (over8)
    {
        brr += brr & 1;
    }

    usart->BRR = brr;

    this->baudrate = baudrate;
}

int USART::read(void *buffer, size_t nBytes)
{
    miosix::Lock<miosix::FastMutex> l(rxMutex);

    char *buf     = reinterpret_cast<char *>(buffer);
    size_t result = 0;
    miosix::FastInterruptDisableLock dLock;
    for (;;)
    {
        // Try to get data from the queue
        for (; result < nBytes; result++)
        {
            if (rxQueue.tryGet(buf[result]) == false)
                break;
            // This is here just not to keep IRQ disabled for the whole loop
            miosix::FastInterruptEnableLock eLock(dLock);
        }

        // Not checking if the nBytes read are more than 0
        if (result == nBytes || (idle && result > 0))
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

    return result;
}

int USART::write(void *buffer, size_t nBytes)
{
    miosix::Lock<miosix::FastMutex> l(txMutex);

    // TODO: Use the send complete interrupt in order not to have a busy while
    // loop waiting
    const char *buf = reinterpret_cast<const char *>(buffer);
    size_t i;
    for (i = 0; i < nBytes; i++)
    {
        while ((usart->SR & USART_SR_TXE) == 0)
            ;

        usart->DR = *buf;
        buf++;
    }

    return i;
}

int USART::writeString(const char *buffer)
{
    int i = 0;
    miosix::Lock<miosix::FastMutex> l(txMutex);

    // Send everything, also the ending '\0' character
    usart->DR = *buffer;
    i++;
    while (*buffer != '\0')
    {
        buffer++;
        while (!(usart->SR & USART_SR_TXE))
            ;

        usart->DR = *buffer;
        i++;
    };

    return i;
}

void USART::clearQueue() { rxQueue.reset(); }

STM32SerialWrapper::STM32SerialWrapper(USARTType *usart, Baudrate baudrate)
{
    this->usart    = usart;
    this->baudrate = baudrate;
    switch (reinterpret_cast<uint32_t>(usart))
    {
        case USART1_BASE:
            this->id = 1;
            initPins(u1tx1::getPin(), 7, u1rx1::getPin(), 7);
            this->serialPortName = std::string("usart1");
            break;
        case USART2_BASE:
            this->id = 2;
            initPins(u2tx1::getPin(), 7, u2rx1::getPin(), 7);
            this->serialPortName = std::string("usart2");
            break;
        case USART3_BASE:
            this->id = 3;
            initPins(u3tx1::getPin(), 7, u3rx1::getPin(), 7);
            this->serialPortName = std::string("usart3");
            break;
        case UART4_BASE:
            this->id = 4;
            initPins(u4tx1::getPin(), 8, u4rx1::getPin(), 8);
            this->serialPortName = std::string("uart4");
            break;
    }
    initialized = false;
    fd          = -1;
}

STM32SerialWrapper::STM32SerialWrapper(USARTType *usart, Baudrate baudrate,
                                       miosix::GpioPin tx, miosix::GpioPin rx)
{
    this->usart    = usart;
    this->baudrate = baudrate;
    switch (reinterpret_cast<uint32_t>(usart))
    {
        case USART1_BASE:
            this->id             = 1;
            this->serialPortName = std::string("usart1");
            break;
        case USART2_BASE:
            this->id             = 2;
            this->serialPortName = std::string("usart2");
            break;
        case USART3_BASE:
            this->id             = 3;
            this->serialPortName = std::string("usart3");
            break;
        case UART4_BASE:
            this->id             = 4;
            this->serialPortName = std::string("uart4");
            break;
    }

    if (id < 4)
        initPins(tx, 7, rx, 7);
    else
        initPins(tx, 8, rx, 8);

    initialized = false;
    fd          = -1;
}

STM32SerialWrapper::~STM32SerialWrapper()
{
    miosix::intrusive_ref_ptr<miosix::DevFs> devFs =
        miosix::FilesystemManager::instance().getDevFs();
    close(fd);
    devFs->remove(serialPortName.c_str());
}

bool STM32SerialWrapper::init()
{
    if (id > 4)
    {
        TRACE(
            "[STM32SerialWrapper] USART id greater than 3 is not supported\n");
        return false;
    }

    if (initialized)
    {
        TRACE(
            "[STM32SerialWrapper] Error : serial communication already "
            "initialized!\n");
        return false;
    }
    else if (!serialCommSetup())
    {
        TRACE(
            "[STM32SerialWrapper] Error : can't initialize serial "
            "communication!\n");
        return false;
    }

    initialized = true;
    return true;
}

bool STM32SerialWrapper::serialCommSetup()
{
    // Creates and adds the serial port to the devices
    if (!pinInitialized)
        serial = new miosix::STM32Serial(id, static_cast<int>(baudrate));
    else
    {
        serial =
            new miosix::STM32Serial(id, static_cast<int>(baudrate), tx, rx);
    }

    // Adds a device to the file system
    if (!miosix::FilesystemManager::instance().getDevFs()->addDevice(
            serialPortName.c_str(),
            miosix::intrusive_ref_ptr<miosix::Device>(serial)))
        return false;

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

bool STM32SerialWrapper::initPins(miosix::GpioPin tx, int nAFtx,
                                  miosix::GpioPin rx, int nAFrx)
{
    if (pinInitialized)
        return false;

    miosix::FastInterruptDisableLock dLock;

    this->tx = tx;
    this->rx = rx;

    tx.mode(miosix::Mode::ALTERNATE);
    tx.alternateFunction(nAFtx);

    rx.mode(miosix::Mode::ALTERNATE);
    rx.alternateFunction(nAFrx);

    pinInitialized = true;
    return true;
}

int STM32SerialWrapper::writeString(const char *data)
{
    // strlen + 1 in order to send the '/0' terminated string
    return ::write(fd, data, strlen(data) + 1);
}

int STM32SerialWrapper::write(void *buf, size_t nChars)
{
    return ::write(fd, buf, nChars);
}

int STM32SerialWrapper::read(void *buf, size_t nBytes)
{
    return ::read(fd, buf, nBytes);
}

}  // namespace Boardcore
