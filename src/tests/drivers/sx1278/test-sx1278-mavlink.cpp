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

#include <drivers/interrupt/external_interrupts.h>
#include <radio/SX1278/SX1278.h>

#include <thread>

#include "common.h"

// Ignore warnings, as we don't want to change third party generated files to
// fix them
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include <mavlink_lib/pyxis/mavlink.h>
#pragma GCC diagnostic pop

#include <radio/MavlinkDriver/MavlinkDriver.h>

using namespace Boardcore;
using namespace miosix;

constexpr uint32_t RADIO_PKT_LENGTH     = 63;
constexpr uint32_t RADIO_OUT_QUEUE_SIZE = 10;
constexpr uint32_t RADIO_MAV_MSG_LENGTH = MAVLINK_MAX_DIALECT_PAYLOAD_SIZE;
constexpr size_t MAV_OUT_BUFFER_MAX_AGE = 200;
constexpr uint32_t FLIGHT_TM_PERIOD     = 250;
constexpr uint32_t STATS_TM_PERIOD      = 2000;

// Mavlink out buffer with 10 packets, 256 bytes each.
using Mav =
    MavlinkDriver<RADIO_PKT_LENGTH, RADIO_OUT_QUEUE_SIZE, RADIO_MAV_MSG_LENGTH>;

Mav* channel;
SX1278* sx1278 = nullptr;

#if defined _BOARD_STM32F429ZI_SKYWARD_GS_V2
#include "interfaces-impl/hwmapping.h"

#define USE_RA01_PC13

#ifdef USE_RA01_PC13  // use ra01
using cs   = peripherals::ra01::pc13::cs;
using dio0 = peripherals::ra01::pc13::dio0;
#else
using cs   = peripherals::ra01::pe4::cs;
using dio0 = peripherals::ra01::pe4::dio0;
#endif

#define SX1278_SPI SPI4

#else
#error "Target not supported"
#endif

#if defined _BOARD_STM32F429ZI_SKYWARD_GS_V2
#ifdef USE_RA01_PC13
void __attribute__((used)) EXTI6_IRQHandlerImpl()
#else
void __attribute__((used)) EXTI3_IRQHandlerImpl()
#endif
#else
#error "Target not supported"
#endif
{
    if (sx1278)
        sx1278->handleDioIRQ();
}

void initBoard()
{
#if defined _BOARD_STM32F429ZI_SKYWARD_GS_V2
    GpioPin dio0 = dio0::getPin();
    enableExternalInterrupt(dio0.getPort(), dio0.getNumber(),
                            InterruptTrigger::RISING_EDGE);
#else
#error "Target not supported"
#endif
}

void onReceive(Mav* channel, const mavlink_message_t& msg)
{
    // Prepare ack messages
    mavlink_message_t ackMsg;
    mavlink_msg_ack_tm_pack(1, 1, &ackMsg, msg.msgid, msg.seq);

    // Send the ack back to the sender
    channel->enqueueMsg(ackMsg);
}

void flightTmLoop()
{
    while (1)
    {
        long long start = miosix::getTick();

        mavlink_message_t msg;
        mavlink_rocket_flight_tm_t tm = {0};
        mavlink_msg_rocket_flight_tm_encode(171, 96, &msg, &tm);

        channel->enqueueMsg(msg);

        Thread::sleepUntil(start + FLIGHT_TM_PERIOD);
    }
}

void statsTmLoop()
{
    while (1)
    {
        long long start = miosix::getTick();

        mavlink_message_t msg;
        mavlink_rocket_stats_tm_t tm = {0};
        mavlink_msg_rocket_stats_tm_encode(171, 96, &msg, &tm);

        channel->enqueueMsg(msg);

        Thread::sleepUntil(start + STATS_TM_PERIOD);
    }
}

int main()
{
    initBoard();

    // Run default configuration
    SX1278::Config config = {.freq_rf  = 412000000,
                             .freq_dev = 25000,
                             .bitrate  = 19200,
                             .rx_bw    = SX1278::RxBw::HZ_83300,
                             .afc_bw   = SX1278::RxBw::HZ_125000,
                             .ocp      = 120,
                             .power    = 17,
                             .shaping  = SX1278::Shaping::GAUSSIAN_BT_1_0,
                             .dc_free  = SX1278::DcFree::WHITENING};

    SX1278::Error err;

    SPIBus bus(SX1278_SPI);
    GpioPin cs = cs::getPin();

    sx1278 = new SX1278(bus, cs);

    printf("\n[sx1278] Configuring sx1278...\n");
    if ((err = sx1278->init(config)) != SX1278::Error::NONE)
    {
        printf("[sx1278] sx1278->init error: %s\n", stringFromErr(err));

        while (1)
            Thread::wait();
    }

    printConfig(config);

    channel = new Mav(sx1278, &onReceive, 0, MAV_OUT_BUFFER_MAX_AGE);
    channel->start();

    std::thread flight_tm_loop([]() { flightTmLoop(); });
    std::thread stats_tm_loop([]() { statsTmLoop(); });

    while (1)
        Thread::wait();

    return 0;
}
