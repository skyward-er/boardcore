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
/*
 * Constructor, initializes the serial port using the default pins, which
 * are:
 * USART1: tx=PA9  rx=PA10 cts=PA11 rts=PA12
 * USART2: tx=PA2  rx=PA3  cts=PA0  rts=PA1
 * USART3: tx=PB10 rx=PB11 cts=PB13 rts=PB14
 */

// A nice feature of the stm32 is that the USART are connected to the same
// GPIOS in all families, stm32f1, f2, f4 and l1. Additionally, USART1 is
// always connected to the APB2, while USART2 and USART3 are always on APB1
// Unfortunately, this does not hold with DMA.
using namespace miosix;

typedef Gpio<GPIOA_BASE, 9> u1tx;
typedef Gpio<GPIOA_BASE, 10> u1rx;
typedef Gpio<GPIOA_BASE, 11> u1cts;
typedef Gpio<GPIOA_BASE, 12> u1rts;

typedef Gpio<GPIOA_BASE, 2> u2tx;
typedef Gpio<GPIOA_BASE, 3> u2rx;
typedef Gpio<GPIOA_BASE, 0> u2cts;
typedef Gpio<GPIOA_BASE, 1> u2rts;

typedef Gpio<GPIOB_BASE, 10> u3tx;
typedef Gpio<GPIOB_BASE, 11> u3rx;
typedef Gpio<GPIOB_BASE, 13> u3cts;
typedef Gpio<GPIOB_BASE, 14> u3rts;

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
    // ledOn();
    saveContext();
    asm volatile("bl _Z22usart1irqImplBoardcorev");
    restoreContext();
    // ledOff();
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
    // ledOn();
    saveContext();
    asm volatile("bl _Z22usart2irqImplBoardcorev");
    restoreContext();
    // ledOff();
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
    // ledOn();
    saveContext();
    asm volatile("bl _Z22usart3irqImplBoardcorev");
    restoreContext();
    // ledOff();
}

namespace Boardcore
{

void USART::IRQHandleInterrupt()
{
    unsigned int status = this->usart->SR;
    char c;
    ledOn();

    // if read data register is empty then read data
    if (status & USART_SR_RXNE)
    {
        // Always read data, since this clears interrupt flags
        c = usart->DR;
        // If no error put data in buffer
        if (!(status & USART_SR_FE))
            if (rxQueue.tryPut(c) == false) /*fifo overflow*/
                ;
    }

    if (status & USART_SR_IDLE)
    {
        c = usart->DR;  // clears interrupt flags
    }

    // ledOff();
}

USART::USART(USARTType *usart, Baudrate baudrate) : usart(usart), rxQueue(64)
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

    {
        miosix::FastInterruptDisableLock dLock;

        // enable usart, receiver, receiver interurpt and idle interrupt
        usart->CR1 = USART_CR1_UE        // Enable port
                     | USART_CR1_RXNEIE  // Interrupt on data received
                     | USART_CR1_IDLEIE  // Interrupt on idle line
                     | USART_CR1_TE      // Transmission enbled
                     | USART_CR1_RE;     // Reception enabled

        // sample only one bit
        usart->CR3 |= USART_CR3_ONEBIT;

        setBaudrate(baudrate);
    }

    printf("init usart %d\n", this->id);
    printf("br: %ld\n", usart->BRR);
}

void USART::init()
{
    if (id > 3 || id < 1)
    {
        return;
    }

    InterruptDisableLock dLock;

    switch (id)
    {
        case 1:
            NVIC_SetPriority(USART1_IRQn,
                             15);  // Lowest priority for serial
            NVIC_EnableIRQ(USART1_IRQn);
            u1tx::getPin().mode(miosix::Mode::ALTERNATE);  // check 1
            u1rx::getPin().mode(miosix::Mode::ALTERNATE);  // check 4
            u1tx::alternateFunction(7);
            u1rx::alternateFunction(7);
            break;
        case 2:
            NVIC_SetPriority(USART2_IRQn,
                             15);  // Lowest priority for serial
            NVIC_EnableIRQ(USART2_IRQn);
            u2tx::getPin().mode(miosix::Mode::ALTERNATE);
            u2rx::getPin().mode(miosix::Mode::ALTERNATE);
            u2tx::alternateFunction(7);
            u2rx::alternateFunction(7);
            break;
        case 3:
            NVIC_SetPriority(USART3_IRQn,
                             15);  // Lowest priority for serial
            NVIC_EnableIRQ(USART3_IRQn);
            u3tx::getPin().mode(miosix::Mode::ALTERNATE);
            u3rx::getPin().mode(miosix::Mode::ALTERNATE);
            u3tx::alternateFunction(7);
            u3rx::alternateFunction(7);
            break;
    }

    // add to the array of usarts so that the interrupts can see it
    ports[id - 1] = this;
}

void USART::setWordLength(WordLength wordLength)
{
    (wordLength == WordLength::BIT8 ? usart->CR1 &= ~USART_CR1_M
                                    : usart->CR1 |= USART_CR1_M);
    this->wordLength = wordLength;
}

void USART::setParity(ParityBit parity)
{
    (parity == ParityBit::NO_PARITY ? usart->CR1 &= ~USART_CR1_PCE
                                    : usart->CR1 |= USART_CR1_PCE);
    this->parity = parity;
}

void USART::setStopBits(int stopBits) { this->stopBits = stopBits; }

void USART::setOversampling(bool oversampling) { this->over8 = oversampling; }

void USART::setBaudrate(Baudrate baudrate)
{
    // fixed point: mantissa first 12 bits;
    // other 4 bits are the fixed point value of the fraction.
    // if over8==0 => DIV_Fraction[3:0] (all fraction used) USART_DIV =
    // f/(16*baud) if over8==1 => 0+DIV_Fraction[2:0] (first bit of fraction is
    // 0)  USART_DIV = f/(8*baud)

    // USART1 is always connected to the APB2, while USART2 and USART3 are
    // always on APB1
    uint32_t f = ClockUtils::getAPBFrequency(
        (id == 1 ? ClockUtils::APB::APB2
                 : ClockUtils::APB::APB1));  // 42MHz clock frequency for APB1,
                                             // 84MHz for APB2

    // <<4 in order to shift to left of 4 positions, to create a fixed point
    // number of 4 decimal digits /8 == >>3 in order to divide per 8 (from the
    // formula in the datasheet)

    uint32_t USART_DIV = ((f << 1) / (((int)baudrate * (over8 ? 1 : 2))));

    uint32_t brr =
        (USART_DIV / 2) + (USART_DIV & 1);  // rounding to the nearest

    if (over8)
    {
        brr += brr & 1;
    }

    usart->BRR = brr;

    // usart_brr register: 12 bit mantissa + 4 bit fraction
    this->baudrate = baudrate;
}

int USART::read(char *buf, int nChars)
{
    printf("SR: %lx\n", usart->SR);
    printf("BR: %lx\n", usart->BRR);
    printf("C1: %lx\n", usart->CR1);
    if (rxQueue.isEmpty())
    {
        printf("no data\n");
    }

    int i;
    for (i = 0; i < nChars; i++)
    {
        if (!rxQueue.tryGet(buf[i]))
        {
            break;
        }
    }
    return i;
}
}  // namespace Boardcore