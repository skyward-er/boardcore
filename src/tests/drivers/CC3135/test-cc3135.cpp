/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include <drivers/interrupt/external_interrupts.h>
#include <radio/CC3135/CC3135.h>

using namespace Boardcore;
using namespace miosix;

/*
Connection diagram:
cc3135:CC_SPI_CS   -> stm32:pa1
cc3135:CC_nHIB     -> stm32:pc14
cc3135:CC_IRQ      -> stm32:pc15
cc3135:CC_SPI_DIN  -> stm32:pc12 (SPI3_MOSI)
cc3135:CC_SPI_DOUT -> stm32:pc11 (SPI3_MISO)
cc3135:CC_SPI_CLK  -> stm32:pc10 (SPI3_SCK)
*/

using tx  = Gpio<GPIOA_BASE, 2>;
using rx  = Gpio<GPIOA_BASE, 3>;
using irq = Gpio<GPIOA_BASE, 4>;
using hib = Gpio<GPIOA_BASE, 5>;

#define CC3135_UART USART2
#define CC3135_HIB

CC3135 *cc3135 = nullptr;

void __attribute__((used)) EXTI4_IRQHandlerImpl()
{
    if (cc3135)
        cc3135->handleIntr();
}

void initBoard()
{

#ifdef CC3135_HIB
    {
        miosix::FastInterruptDisableLock dLock;

        hib::mode(miosix::Mode::OUTPUT);
    }

    hib::high();
#endif

#ifdef CC3135_UART
    {
        miosix::FastInterruptDisableLock dLock;

        irq::mode(miosix::Mode::INPUT);
        tx::mode(miosix::Mode::ALTERNATE);
        tx::alternateFunction(7);
        rx::mode(miosix::Mode::ALTERNATE);
        rx::alternateFunction(7);
    }

    auto irq_pin = irq::getPin();
    enableExternalInterrupt(irq_pin.getPort(), irq_pin.getNumber(),
                            InterruptTrigger::RISING_EDGE);
#endif
}

int main()
{
    initBoard();

#ifdef CC3135_UART
    std::unique_ptr<ICC3135Iface> iface(new CC3135Uart(CC3135_UART));
#endif

    cc3135 = new CC3135(std::move(iface));

    Thread::sleep(200);

    cc3135->getVersion();
    while (true)
    {
        cc3135->dummyRead();
    }
}
