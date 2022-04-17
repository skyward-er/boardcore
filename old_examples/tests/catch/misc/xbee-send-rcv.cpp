/* Copyright (c) 2019 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

#include <drivers/spi/SPIDriver.h>
#include <interfaces-impl/hwmapping.h>
#include <miosix.h>
#include <radio/Xbee/Xbee.h>
#include <utils/Stats/Stats.h>

#include <cstdio>
#include <iostream>
#include <string>

using std::cin;
using std::cout;
using std::string;

static const unsigned int PKT_SIZE = 128;

using namespace miosix;
using namespace interfaces;

// WARNING: If flashing on stm32f49 discovery board (with screen removed) use
// SPI1 as the 2nd isnt working.

// Discovery
SPIBus bus{SPI1};
GpioPin cs   = sensors::lsm6ds3h::cs::getPin();
GpioPin attn = xbee::attn::getPin();
GpioPin rst  = xbee::reset::getPin();

// Death stack
// SPIBus bus{SPI2};
// GpioPin cs = xbee::cs::getPin();
// GpioPin attn = xbee::attn::getPin();
// GpioPin rst = xbee::reset::getPin();

Xbee::Xbee* xbee_transceiver;

void __attribute__((used)) EXTI10_IRQHandlerImpl()
{
    Xbee::handleATTNInterrupt();
}

void enableXbeeInterrupt()
{
    {
        FastInterruptDisableLock l;
        RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    }
    // Refer to the datasheet for a detailed description on the procedure and
    // interrupt registers

    // Clear the mask on the wanted line
    EXTI->IMR |= EXTI_IMR_MR10;

    // Trigger the interrupt on a falling edge
    EXTI->FTSR |= EXTI_FTSR_TR10;

    // Trigger the interrupt on a rising edge
    // EXTI->RTSR |= EXTI_RTSR_TR0;

    EXTI->PR |= EXTI_PR_PR10;  // Reset pending register

    // Enable interrupt on PF10 in SYSCFG
    SYSCFG->EXTICR[2] = 0x5U << 8;

    // Enable the interrput in the interrupt controller
    NVIC_EnableIRQ(EXTI15_10_IRQn);
    NVIC_SetPriority(EXTI15_10_IRQn, 15);
}

void send()
{
    uint8_t buf[PKT_SIZE];
    buf[0]            = '/';
    buf[PKT_SIZE - 1] = '\\';

    int fail = 0;
    char c   = 48;
    for (;;)
    {
        memset(buf + 1, c++, PKT_SIZE - 2);
        if (c > 122)
        {
            c = 48;
        }

        if (!xbee_transceiver->send(buf, PKT_SIZE))
        {
            printf("[%d] Send error %d\n", (int)getTick(), ++fail);
        }
        else
        {
            if (c % 5 == 0)
            {
                printf("Send ok.\n");
            }
        }

        Thread::sleep(1000);
    }
}

void receive(void*)
{
    uint8_t buf[512];
    for (;;)
    {
        ssize_t len = xbee_transceiver->receive(buf, 512);
        if (len <= 0)
        {
            printf("Receive failed.\n");
        }
        else
        {
            for (ssize_t i = 0; i < len; i++)
            {
                // Only print displayable ascii chars
                if (buf[i] >= 32 && buf[i] <= 126)
                {
                    printf("%c", buf[i]);
                }
            }
            printf("\n");
            for (ssize_t i = 0; i < len; i++)
            {
                printf("%02X ", buf[0]);
            }
            printf("\n\n");
        }
    }
}

void resetx()
{
    xbee::reset::mode(Mode::OPEN_DRAIN);
    xbee::reset::low();
    delayUs(500);
    xbee::reset::high();
    // xbee::reset::mode(Mode::INPUT);
}

int main()
{
    enableXbeeInterrupt();
    resetx();

    // xbee::sleep_req::mode(Mode::OUTPUT);
    // xbee::sleep_req::low();

    // reset();

    xbee_transceiver = new Xbee::Xbee(bus, cs, attn, rst);

    xbee_transceiver->start();

    // Send & receive
    Thread::create(receive, 2048);
    // for(;;)
    //     Thread::sleep(1000);
    send();

    return 0;
}
