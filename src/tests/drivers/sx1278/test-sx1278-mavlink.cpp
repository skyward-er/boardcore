/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Alvise de'Faveri Tron, Nuno Barcellos, Davide Mor
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

#include <drivers/SX1278/SX1278.h>
#include <drivers/interrupt/external_interrupts.h>

#include "test-sx1278-core.h"

// Ignore warnings, as we don't want to change third party generated files to
// fix them
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include <mavlink_lib/lynx/mavlink.h>
#pragma GCC diagnostic pop

#include <radio/MavlinkDriver/MavlinkDriver.h>

using namespace Boardcore;
using namespace miosix;

static const unsigned int queue_len          = 10;
static const unsigned int packet_size        = 256;
static const unsigned int silence_after_send = 250;
static const unsigned int max_pkt_age        = 1000;
static const unsigned int ping_period        = 1000;

// Mavlink out buffer with 10 packets, 256 bytes each.
using Mav = MavlinkDriver<packet_size, queue_len>;

Mav* channel;

SPIBus bus(SPI3);

GpioPin sck(GPIOC_BASE, 10);
GpioPin miso(GPIOC_BASE, 11);
GpioPin mosi(GPIOC_BASE, 12);
GpioPin cs(GPIOA_BASE, 1);
GpioPin dio(GPIOC_BASE, 15);

SX1278* sx1278 = nullptr;

void __attribute__((used)) EXTI15_IRQHandlerImpl()
{
    if (sx1278)
        sx1278->handleDioIRQ();
}

/// Initialize stm32f407g board.
void initBoard()
{
    {
        miosix::FastInterruptDisableLock dLock;

        // Enable SPI3
        RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
        RCC_SYNC();

        // Setup SPI pins
        sck.mode(miosix::Mode::ALTERNATE);
        sck.alternateFunction(6);
        miso.mode(miosix::Mode::ALTERNATE);
        miso.alternateFunction(6);
        mosi.mode(miosix::Mode::ALTERNATE);
        mosi.alternateFunction(6);

        cs.mode(miosix::Mode::OUTPUT);
        dio.mode(miosix::Mode::INPUT);
    }

    cs.high();
    enableExternalInterrupt(dio.getPort(), dio.getNumber(),
                            InterruptTrigger::RISING_EDGE);
}

/**
 * @brief Receive function: print the received message id and send an ACK.
 */
static void onReceive(Mav* channel, const mavlink_message_t& msg)
{
    printf("[TmtcTest] Received message, id: %d! Sending ack\n", msg.msgid);

    // Prepare ack messages
    mavlink_message_t ackMsg;
    mavlink_msg_ack_tm_pack(1, 1, &ackMsg, msg.msgid, msg.seq);

    // Send the ack back to the sender
    bool ackSent = channel->enqueueMsg(ackMsg);

    if (!ackSent)
        printf("[Receiver] Could not enqueue ack\n");
}

/**
 * @brief This test enqueues a ping message every second and replies to every
 * received message with an ACK.
 */
int main()
{
    initBoard();

    SX1278::Config config;
    SX1278::Error err;

    sx1278 = new SX1278(bus, cs);

    printf("\n[sx1278] Configuring sx1278...\n");
    printConfig(config);
    if ((err = sx1278->init(config)) != SX1278::Error::NONE)
    {
        printf("[sx1278] sx1278->init error: %s\n", stringFromErr(err));
        return -1;
    }

    channel = new Mav(sx1278, &onReceive, silence_after_send, max_pkt_age);

    channel->start();

    // Send function: enqueue a ping every second
    while (1)
    {
        printf("[TmtcTest] Enqueueing ping\n");

        // Create a Mavlink message
        mavlink_message_t pingMsg;
        mavlink_msg_ping_tc_pack(1, 1, &pingMsg, miosix::getTick());

        // Send the message
        bool ackSent = channel->enqueueMsg(pingMsg);

        if (!ackSent)
            printf("[TmtcTest] Could not enqueue ping\n");

        // LED blink
        ledOn();
        miosix::delayMs(200);
        ledOff();

        miosix::Thread::sleep(ping_period);
    }

    return 0;
}
