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

#include <drivers/BusTemplate.h>

using std::cin;
using std::cout;
using std::string;

using HwTimer                         = HardwareTimer<uint32_t, 2>;
static const unsigned int NUM_PACKETS = 250;
static unsigned int PKT_SIZE          = 128;

using namespace miosix;
using namespace interfaces;

// SPI1 binding al sensore
typedef BusSPI<2, spi2::mosi, spi2::miso, spi2::sck> busSPI2;  // Creo la SPI2

using ATTN = Gpio<GPIOF_BASE, 10>;
using cs   = Gpio<GPIOF_BASE, 9>;

typedef Xbee::Xbee<busSPI2, cs, ATTN, xbee::reset> Xbee_t;
Xbee_t xbee_transceiver;

void printStat(const char* title, StatsResult r)
{
    printf("%s:\nMin: %.3f\nMax: %.3f\nMean: %.3f\nStddev:  %.3f\n\n", title,
           r.minValue, r.maxValue, r.mean, r.stdev);
}

void __attribute__((used)) EXTI10_IRQHandlerImpl() { Xbee::handleInterrupt(); }

void enableXbeeInterrupt()
{
    ATTN::mode(Mode::INPUT_PULL_UP);

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

void reset();

void send2()
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

        if (!xbee_transceiver.send(buf, PKT_SIZE))
        {
            printf("[%d] Send error %d\n", (int)getTick(), ++fail);
            reset();
        }
        else
        {
            if (c % 5 == 0)
            {
                printf("Send ok.\n");
            }
        }
    }
}

void send()
{
    printf("Packet size: %d\n", PKT_SIZE);
    Stats times_stat{};
    Stats rates_stat{};

    HwTimer& t = HwTimer::instance();
    t.start();

    uint32_t send_tick_acc = 0;

    int send_fail_cnt = 0;
    uint8_t buf[PKT_SIZE];
    vector<Xbee::XbeeStatus> fail_status;

    for (unsigned int i = 0; i < NUM_PACKETS; i++)
    {
        // Fill the buffer
        memset(buf, (uint8_t)i, PKT_SIZE);

        uint32_t send_tick = t.tick();
        // Send message
        if (!xbee_transceiver.send(buf, PKT_SIZE))
        {
            fail_status.push_back(xbee_transceiver.getStatus());
            if ((*fail_status.end()).tx_timeout_count > 0)
            {
                break;
            }
        }
        // Time to send
        send_tick = t.tick() - send_tick;

        float byterate = PKT_SIZE * 1000 / t.toMilliSeconds(send_tick);
        // Stat
        times_stat.add(t.toMilliSeconds(send_tick));
        rates_stat.add(byterate);

        // Total time to send
        send_tick_acc += send_tick;
    }

    if (fail_status.size() > 0)
    {
        for (auto s : fail_status)
        {
            s.print();
        }
    }

    printStat("Times", times_stat.getStats());
    printStat("Rates", rates_stat.getStats());

    const int bytes_sent = PKT_SIZE * NUM_PACKETS;

    float total_ms_send = t.toMilliSeconds(send_tick_acc);

    float byterate = bytes_sent * 1000 / total_ms_send;
    printf("Mean:\nByterate: %.3f.\tBitrate: %.3f\n\n", byterate, byterate * 8);

    printf("Fail cnt: %d\n", send_fail_cnt);
}

void receive()
{
    printf("Packet size: %d\n", PKT_SIZE);
    uint8_t last_pkt_id = 0;

    Stats times_stat{};
    Stats rates_stat{};

    HwTimer& t = HwTimer::instance();
    t.start();

    uint32_t rcv_tick_acc = 0;

    int pkt_lost = 0;
    uint8_t buf[PKT_SIZE];

    for (unsigned int i = 0; i < NUM_PACKETS; i++)
    {
        uint32_t tick = t.tick();
        xbee_transceiver.receive(buf, PKT_SIZE);
        tick = t.tick() - tick;
        rcv_tick_acc += tick;
        float byterate = PKT_SIZE * 1000 / t.toMilliSeconds(tick);
        times_stat.add(t.toMilliSeconds(tick));
        rates_stat.add(byterate);

        // printf("%c\n", d);

        pkt_lost += buf[0] - last_pkt_id;
        last_pkt_id = buf[0] + 1;
    }

    printStat("Times", times_stat.getStats());
    printStat("Rates", rates_stat.getStats());

    const int bytes_sent = PKT_SIZE * NUM_PACKETS;

    float total_ms_send = t.toMilliSeconds(rcv_tick_acc);

    float byterate = bytes_sent * 1000 / total_ms_send;
    printf("Mean:\nByterate: %.3f.\tBitrate: %.3f\n\n", byterate, byterate * 8);

    printf("Packet loss: %d\n", pkt_lost);
}

// void receive()
// {
//     for (;;)
//     {
//         uint8_t buf[PKT_SIZE];
//         if (xbee_transceiver.receive(buf, PKT_SIZE))
//         {
//             printf("recv ok\n");
//         }
//         else
//         {
//             printf("recv failed\n");
//         }
//     }
// }

void rcv(void*)
{
    uint8_t buf[PKT_SIZE];
    for (;;)
    {
        ssize_t len = xbee_transceiver.receive(buf, PKT_SIZE);
        for (ssize_t i = 0; i < len; i++)
        {
            printf("%02X\n", buf[0]);
        }
        if (len == 0)
        {
            printf("Receive failed.\n");
        }
    }
}

void reset()
{
    xbee::reset::mode(Mode::OUTPUT);
    xbee::reset::low();
    delayUs(100);
    xbee::reset::mode(Mode::OPEN_DRAIN);
}

int main()
{
  //  xbee::sleep_req::mode(Mode::OUTPUT);
    //xbee::sleep_req::low();

   // xbee::reset::mode(Mode::OPEN_DRAIN);

    HwTimer& t = HwTimer::instance();
    t.setPrescaler(1024);

    enableXbeeInterrupt();
    busSPI2::init();

    // Enable SPI
    cs::low();
    Thread::sleep(10);
    cs::high();
    Thread::sleep(10);
    

    xbee_transceiver.start();

    send2();

    /* while(true)
     {
         std::string s;
         cin >> s;

         vector<uint8_t> buf(s.begin(), s.end());


         xbee_transceiver.send(buf.data(), buf.size());
     }*/
    /*uint8_t snd[PKT_SIZE];

    uint8_t i = 'a';
    printf("Sending...\n");
    for (;;)
    {

        memset(snd, i++, PKT_SIZE);
        if (i > 'z')
            i = 'a';
        if (!xbee_transceiver.send(snd, PKT_SIZE))
        {
            // printf("Send error.\n");
        }
    }*/

    /*uint8_t buf[PKT_SIZE];
    for (;;)
    {
        ssize_t len = xbee_transceiver.receive(buf, PKT_SIZE);
        if (len > 0)
        {
            printf("%c (%d)\n", buf[0], len);
        }
        else
        {
            printf("Receive failed.\n");
        }
    }*/

    cout << "(S)end or (R)eceive?\n";
    char c;
    cin >> &c;

    if (c == 'S' || c == 's')
    {
        PKT_SIZE = 16;
        send();
        PKT_SIZE = 32;
        send();
        PKT_SIZE = 64;
        send();
        PKT_SIZE = 128;
        send();
        PKT_SIZE = 256;
        send();
    }
    else if (c == 'R' || c == 'r')
    {
        PKT_SIZE = 16;
        receive();
        PKT_SIZE = 32;
        receive();
        PKT_SIZE = 64;
        receive();
        PKT_SIZE = 128;
        receive();
        PKT_SIZE = 256;
        receive();
    }

    cout << "End \n";

    for (;;)
        ;
}