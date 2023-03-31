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
#include <radio/SX1278/SX1278Fsk.h>

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

#if defined _BOARD_STM32F429ZI_SKYWARD_GS_V2
#include "interfaces-impl/hwmapping.h"

using cs   = peripherals::ra01::pc13::cs;
using dio0 = peripherals::ra01::pc13::dio0;
using dio1 = peripherals::ra01::pc13::dio1;
using dio3 = peripherals::ra01::pc13::dio3;

using sck  = interfaces::spi4::sck;
using miso = interfaces::spi4::miso;
using mosi = interfaces::spi4::mosi;

#define SX1278_SPI SPI4

#define SX1278_IRQ_DIO0 EXTI6_IRQHandlerImpl
#define SX1278_IRQ_DIO1 EXTI2_IRQHandlerImpl
#define SX1278_IRQ_DIO3 EXTI11_IRQHandlerImpl

#else
#error "Target not supported"
#endif

SX1278Fsk* sx1278 = nullptr;

void __attribute__((used)) SX1278_IRQ_DIO0()
{
    if (sx1278)
        sx1278->handleDioIRQ(SX1278Fsk::Dio::DIO0);
}

void __attribute__((used)) SX1278_IRQ_DIO1()
{
    if (sx1278)
        sx1278->handleDioIRQ(SX1278Fsk::Dio::DIO1);
}

void __attribute__((used)) SX1278_IRQ_DIO3()
{
    if (sx1278)
        sx1278->handleDioIRQ(SX1278Fsk::Dio::DIO3);
}

void initBoard()
{
    GpioPin dio0_pin = dio0::getPin();
    GpioPin dio1_pin = dio1::getPin();
    GpioPin dio3_pin = dio3::getPin();

    enableExternalInterrupt(dio0_pin.getPort(), dio0_pin.getNumber(),
                            InterruptTrigger::RISING_EDGE);
    enableExternalInterrupt(dio1_pin.getPort(), dio1_pin.getNumber(),
                            InterruptTrigger::RISING_EDGE);
    enableExternalInterrupt(dio3_pin.getPort(), dio3_pin.getNumber(),
                            InterruptTrigger::RISING_EDGE);
}

Mav* channel;

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

    SX1278Fsk::Config config;
    SX1278Fsk::Error err;

    SPIBus bus(SX1278_SPI);
    GpioPin cs = cs::getPin();

    SPIBusConfig spi_config;
    spi_config.clockDivider = SPI::ClockDivider::DIV_64;
    spi_config.mode         = SPI::Mode::MODE_0;
    spi_config.bitOrder     = SPI::BitOrder::MSB_FIRST;

    sx1278 = new SX1278Fsk(SPISlave(bus, cs, spi_config));

    printf("\n[sx1278] Configuring sx1278...\n");
    if ((err = sx1278->init(config)) != SX1278Fsk::Error::NONE)
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
