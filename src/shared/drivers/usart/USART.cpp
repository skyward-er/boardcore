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

#include <miosix.h>

// TODO: define the length of the queue
#define QUEUE_LEN 256

// A nice feature of the stm32 is that the USART are connected to the same
// GPIOS in all families, stm32f1, f2, f4 and l1. Additionally, USART1 is
// always connected to the APB2, while USART2 and USART3 are always on APB1.
// Unfortunately, this does not hold with DMA.

// typedef miosix::Gpio<GPIOA_BASE, 9> u1tx;
// typedef miosix::Gpio<GPIOA_BASE, 10> u1rx;
typedef miosix::Gpio<GPIOB_BASE, 6> u1tx;
typedef miosix::Gpio<GPIOB_BASE, 7> u1rx;
typedef miosix::Gpio<GPIOA_BASE, 11> u1cts;
typedef miosix::Gpio<GPIOA_BASE, 12> u1rts;

typedef miosix::Gpio<GPIOA_BASE, 2> u2tx;
typedef miosix::Gpio<GPIOA_BASE, 3> u2rx;
typedef miosix::Gpio<GPIOA_BASE, 0> u2cts;
typedef miosix::Gpio<GPIOA_BASE, 1> u2rts;

typedef miosix::Gpio<GPIOB_BASE, 10> u3tx;
typedef miosix::Gpio<GPIOB_BASE, 11> u3rx;
typedef miosix::Gpio<GPIOB_BASE, 13> u3cts;
typedef miosix::Gpio<GPIOB_BASE, 14> u3rts;

/// Pointer to serial port classes to let interrupts access the classes
static Boardcore::USART *ports[3] = {0};

/**
 * \internal interrupt routine for usart1 actual implementation
 */
void __attribute__((used)) usart1irqImplBoardcore()
{
    if (ports[0])
        ports[0]->IRQHandleInterrupt();
}

/**
 * \internal interrupt routine for usart1
 */
void __attribute__((naked)) USART1_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z22usart1irqImplBoardcorev");
    restoreContext();
}

/**
 * \internal interrupt routine for usart2 actual implementation
 */
void __attribute__((used)) usart2irqImplBoardcore()
{
    if (ports[1])
        ports[1]->IRQHandleInterrupt();
}

/**
 * \internal interrupt routine for usart2
 */
void __attribute__((naked)) USART2_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z22usart2irqImplBoardcorev");
    restoreContext();
}

/**
 * \internal interrupt routine for usart3 actual implementation
 */
void __attribute__((used)) usart3irqImplBoardcore()
{
    if (ports[2])
        ports[2]->IRQHandleInterrupt();
}

/**
 * \internal interrupt routine for usart3
 */
void __attribute__((naked)) USART3_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z22usart3irqImplBoardcorev");
    restoreContext();
}

namespace Boardcore
{
void USART::IRQHandleInterrupt()
{
    char c;

    // if read data register is empty then read data
    if (usart->SR & USART_SR_RXNE)
    {
        // Always read data, since this clears interrupt flags
        c = usart->DR;
        // If no error put data in buffer
        if (!(usart->SR & USART_SR_FE))
            if (rxQueue.tryPut(c) == false) /*fifo overflow*/
                ;

        idle = false;
    }

    if (usart->SR & USART_SR_IDLE)
        idle = true;

    if (usart->SR & USART_SR_IDLE || rxQueue.size() >= QUEUE_LEN / 2)
    {
        c = usart->DR;  // clears interrupt flags

        // Enough data in buffer or idle line, awake thread
        if (rxWaiting)
        {
            rxWaiting->IRQwakeup();
            rxWaiting = 0;
        }
    }
}

USART::USART(USARTType *usart, Baudrate baudrate)
    : usart(usart), rxQueue(QUEUE_LEN)
{
    // enable the peripehral on the right APB
    ClockUtils::enablePeripheralClock(usart);
    RCC_SYNC();

    switch (reinterpret_cast<uint32_t>(usart))
    {
        case USART1_BASE:
            this->id = 1;
            break;
        case USART2_BASE:
            this->id = 2;
            break;
        case USART3_BASE:
            this->id = 3;
            break;
    }

    // enabling the usart
    {
        miosix::FastInterruptDisableLock dLock;
        usart->CR1 |= USART_CR1_UE;
    }

    // setting the baudrate chosen
    setBaudrate(baudrate);

    // default settings
    setStopBits(1);
    setWordLength(USART::WordLength::BIT8);
    setParity(USART::ParityBit::NO_PARITY);
    setOversampling(false);
}

USART::~USART()
{
    {
        miosix::FastInterruptDisableLock dLock;

        // take out the usart object we are going to destruct
        ports[this->id] = nullptr;

        // disabling the usart
        usart->CR1 &= ~USART_CR1_UE;

        switch (id)
        {
            case 1:
                NVIC_DisableIRQ(USART1_IRQn);
                break;
            case 2:
                NVIC_DisableIRQ(USART2_IRQn);
                break;
            case 3:
                NVIC_DisableIRQ(USART3_IRQn);
                break;
        }
    }
}

bool USART::init()
{
    if (id > 3 || id < 1)
    {
        return false;
    }

    miosix::FastInterruptDisableLock dLock;

    // enable usart, receiver, receiver interurpt and idle interrupt
    usart->CR1 |= USART_CR1_RXNEIE    // Interrupt on data received
                  | USART_CR1_IDLEIE  // interrupt on idle line
                  | USART_CR1_TE      // Transmission enbled
                  | USART_CR1_RE;     // Reception enabled

    // sample only one bit
    usart->CR3 |= USART_CR3_ONEBIT;

    switch (id)
    {
        case 1:
            // Lowest priority for serial
            NVIC_SetPriority(USART1_IRQn, 15);
            NVIC_EnableIRQ(USART1_IRQn);
            u1tx::getPin().mode(miosix::Mode::ALTERNATE);
            u1rx::getPin().mode(miosix::Mode::ALTERNATE);
            u1tx::alternateFunction(7);
            u1rx::alternateFunction(7);
            break;
        case 2:
            // Lowest priority for serial
            NVIC_SetPriority(USART2_IRQn, 15);
            NVIC_EnableIRQ(USART2_IRQn);
            u2tx::getPin().mode(miosix::Mode::ALTERNATE);
            u2rx::getPin().mode(miosix::Mode::ALTERNATE);
            u2tx::alternateFunction(7);
            u2rx::alternateFunction(7);
            break;
        case 3:
            // Lowest priority for serials
            NVIC_SetPriority(USART3_IRQn, 15);
            NVIC_EnableIRQ(USART3_IRQn);
            u3tx::getPin().mode(miosix::Mode::ALTERNATE);
            u3rx::getPin().mode(miosix::Mode::ALTERNATE);
            u3tx::alternateFunction(7);
            u3rx::alternateFunction(7);
            break;
    }

    // add to the array of usarts so that the interrupts can see it
    ports[id - 1] = this;

    return true;
}

void USART::enableDMA()
{
    miosix::FastInterruptDisableLock dLock;
    // enable DMA transmitter and receiver
    usart->CR3 |= USART_CR3_DMAT | USART_CR3_DMAR;
    return;
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

    // USART1 is always connected to the APB2, while USART2 and USART3 are
    // always on APB1
    uint32_t f = ClockUtils::getAPBFrequency(
        (id == 1 ? ClockUtils::APB::APB2     // High speed APB2
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

        // not checking if the nBytes read are more than 0
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
    // TODO: use the send complete interrupt in order not to have a busy while
    // loop waiting
    const char *buf = reinterpret_cast<const char *>(buffer);
    size_t i        = 0;
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

    // send everything, comprising the '\0' character
    usart->DR = *buffer;
    if (*buffer != '\0')
    {
        do
        {
            buffer++;
            while (!(usart->SR & USART_SR_TXE))
                ;

            usart->DR = *buffer;
            i++;
        } while (*buffer != '\0');
    }

    return i;
}

int USART::getId() { return this->id; }
}  // namespace Boardcore