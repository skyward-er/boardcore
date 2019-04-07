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

#include "drivers/HardwareTimer.h"
#include "drivers/Xbee/Xbee.h"
#include "math/Stats.h"

#include <drivers/BusTemplate.h>

using std::cin;
using std::cout;

using HwTimer = HardwareTimer<uint32_t, 2>;
static const unsigned int NUM_PACKETS = 0xFFFF + 1;
static const unsigned int PKT_SIZE    = 256;

using namespace miosix;
using namespace interfaces;

// SPI1 binding al sensore
typedef BusSPI<2, spi2::mosi, spi2::miso, spi2::sck> busSPI2;  // Creo la SPI2
typedef ProtocolSPI<busSPI2, InAir9B::cs>spiProt;      // La lego al Chip Select

using ATTN = Gpio<GPIOF_BASE, 10>;

typedef Xbee<spiProt, ATTN> xbee_t;
xbee_t xbee;

void printStat(const char* title, StatsResult r)
{
    printf("%s:\nMin: %.3f ms\nMax: %.3f ms\nMean: %.3f ms\nStddev:  %.3f\n\n",
           title, r.minValue, r.maxValue, r.mean, r.stdev);
}


void __attribute__((used)) EXTI10_IRQHandler()
{
    XbeeIRQ::handleInterrupt();
}

void enableXbeeInterrupt()
{
    {
        FastInterruptDisableLock l;
        RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    }
    // Refer to the datasheet for a detailed description on the procedure and interrupt registers

    // Clear the mask on the wanted line
    EXTI->IMR |= EXTI_IMR_MR10;

    // Trigger the interrupt on a falling edge
    EXTI->FTSR |= EXTI_FTSR_TR10;

    // Trigger the interrupt on a rising edge
    //EXTI->RTSR |= EXTI_RTSR_TR0;

    EXTI->PR |= EXTI_PR_PR10; // Reset pending register

    // Enable interrupt on PF10 in SYSCFG
    SYSCFG->EXTICR[3] = 0x5U<<8;

    // Enable the interrput in the interrupt controller
    NVIC_EnableIRQ(EXTI15_10_IRQn);
    NVIC_SetPriority(EXTI15_10_IRQn, 15);
}

void send()
{
    Stats times_stat{};
    Stats rates_stat{};

    HwTimer& t = HwTimer::instance();
    t.start();

    float send_tick_acc = 0;

    int send_fail_cnt = 0;
    uint8_t buf[PKT_SIZE];

    for (unsigned int i = 0; i < NUM_PACKETS; i++)
    {
        // Fill the buffer
        for (int j = 0; i < PKT_SIZE; i += 2)
        {
            memcpy(&buf[j], &i, 2);
        }

        uint32_t send_tick = t.tick();
        // Send message
        if (!xbee.send(buf, PKT_SIZE))
        {
            ++send_fail_cnt;
        }
        // Time to send
        send_tick = t.tick() - send_tick;

        float byterate = PKT_SIZE / t.toMilliSeconds(send_tick);
        // Stat
        times_stat.add(t.toMilliSeconds(send_tick));
        rates_stat.add(byterate);

        // Total time to send
        send_tick_acc += send_tick;
    }
    printStat("Times", times_stat.getStats());
    printStat("Rates", times_stat.getStats());

    const int bytes_sent = PKT_SIZE * NUM_PACKETS;

    float total_ms_send = t.toMilliSeconds(send_tick_acc);

    float byterate = bytes_sent / total_ms_send;
    printf("Mean:\nByterate: %.3f.\tBitrate: %.3f\n\n", byterate, byterate * 8);

    printf("Fail cnt: %d\n", send_fail_cnt);
}

void receive()
{
    Stats times_stat{};
    Stats rates_stat{};

    HwTimer& t = HwTimer::instance();
    t.start();

    float rcv_tick_acc = 0;

    int pkt_lost = 0;
    uint8_t buf[PKT_SIZE];

    for (unsigned int i = 0; i < NUM_PACKETS; i++)
    {
        uint32_t tick = t.tick();
        xbee.receive(buf, PKT_SIZE);
        tick = t.tick() - tick;

        rcv_tick_acc += tick;
        float byterate = PKT_SIZE / t.toMilliSeconds(tick);
        times_stat.add(t.toMilliSeconds(tick));
        rates_stat.add(byterate);

        uint16_t d;
        memcpy(&d, buf, 2);

        if(d != i)
        {
            ++pkt_lost;
        }
    }

    printStat("Times", times_stat.getStats());
    printStat("Rates", times_stat.getStats());

    const int bytes_sent = PKT_SIZE * NUM_PACKETS;

    float total_ms_send = t.toMilliSeconds(rcv_tick_acc);

    float byterate = bytes_sent / total_ms_send;
    printf("Mean:\nByterate: %.3f.\tBitrate: %.3f\n\n", byterate, byterate * 8);

    printf("Packet loss: %d\n", pkt_lost);
}

int main()
{
    enableXbeeInterrupt();

    cout << "(S)end or (R)eceive?\n";
    char c;
    cin >> &c;

    if (c == 'S' || c == 's')
    {
        send();
    }
    else if (c == 'R' || c == 'r')
    {
        receive();
    }

    cout << "End \n";
    
    for(;;);
}