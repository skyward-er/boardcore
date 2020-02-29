/*
 * Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <interfaces-impl/hwmapping.h>
#include <miosix.h>
#include <cstdio>
#include <iostream>
#include <string>

#include "drivers/HardwareTimer.h"
#include "drivers/Xbee/Xbee.h"
#include "math/Stats.h"

#include <drivers/spi/SPIDriver.h>

using std::cin;
using std::cout;
using std::string;

static constexpr int MAX_PKT_SIZE = 255;
static constexpr int PKT_NUM = 100;

using namespace miosix;
using namespace interfaces;

// WARNING: If flashing on stm32f49 discovery board (with screen removed) use
// SPI1 as the 2nd isnt working.

// Discovery
SPIBus bus{SPI1};
GpioPin cs = sensors::lsm6ds3h::cs::getPin();
GpioPin attn = xbee::attn::getPin();
GpioPin rst = xbee::reset::getPin();

// Death stack
// SPIBus bus{SPI2};
// GpioPin cs = xbee::cs::getPin();
// GpioPin attn = xbee::attn::getPin();
// GpioPin rst = xbee::reset::getPin();


Xbee::Xbee* xbee_transceiver;

void __attribute__((used)) EXTI10_IRQHandlerImpl() { Xbee::handleATTNInterrupt(); }

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

uint8_t snd_buf[MAX_PKT_SIZE];
int snd_cntr = 0;

bool sendPacket(uint8_t size)
{
    snd_buf[0] = '{';
    snd_buf[size-1] = '}';

    
    for(int i = 0; i < size - 2; i++)
    {
        snd_buf[i+1] = ((snd_cntr + i) % 75 ) + 48; //ASCII char from 0 to z
    }
    ++snd_cntr;

    if (!xbee_transceiver->send(snd_buf, size))
    {
        return false;
    }
    return true;
}

void resetXBee()
{
    xbee::reset::low();
    delayUs(500);
    xbee::reset::high();
}

int main()
{
    enableXbeeInterrupt();
    xbee_transceiver = new Xbee::Xbee(bus, cs, attn, rst);

    xbee_transceiver->start();
    resetXBee();

    printf("XBee bitrate measurement\n");
    printf("Send 100 packets of a certain size (from 16 to 256, step 16) and reporting send time.\n");
    printf("Press enter to start\n");
    string s;
    std::getline(cin, s);

    int pkt_size = 16;
    while(pkt_size <= MAX_PKT_SIZE)
    {
        printf("Testing %d byte packets:\n", pkt_size);
        long long results[PKT_NUM];

        for(int i = 0; i < PKT_NUM; i++)
        {
            long long start = getTick();
            if(!sendPacket(pkt_size))
            {
                printf("Error sending packet %d (size: %d)\n", i, pkt_size);
                goto end;
            }
            results[i] = getTick() - start;
        }
        printf("Results for %d byte packet size:\n");
        for(int i = 0; i < PKT_NUM; i++)
        {
            printf("%d\n", (int)results[i]);
        }
        pkt_size += 16;
        if(pkt_size == 256)
            pkt_size = 255; // Max we can send is 255
    }

end:
    printf("End\n");
    for(;;)
        Thread::sleep(1000);
    return 0;
}