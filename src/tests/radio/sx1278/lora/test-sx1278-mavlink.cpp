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
#include <radio/SX1278/SX1278Frontends.h>
#include <radio/SX1278/SX1278Lora.h>

#include <thread>

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

constexpr uint32_t RADIO_PKT_LENGTH     = 255;
constexpr uint32_t RADIO_OUT_QUEUE_SIZE = 10;
constexpr uint32_t RADIO_MAV_MSG_LENGTH = MAVLINK_MAX_DIALECT_PAYLOAD_SIZE;
constexpr size_t MAV_OUT_BUFFER_MAX_AGE = 0;
constexpr uint16_t SLEEP_AFTER_SEND     = 0;
constexpr uint32_t FLIGHT_TM_PERIOD     = 8000;
constexpr uint32_t STATS_TM_PERIOD      = 1000;

// Mavlink out buffer with 10 packets, 256 bytes each.
using Mav =
    MavlinkDriver<RADIO_PKT_LENGTH, RADIO_OUT_QUEUE_SIZE, RADIO_MAV_MSG_LENGTH>;

#if defined _BOARD_STM32F429ZI_SKYWARD_GS_V2
#include "interfaces-impl/hwmapping.h"

// Uncomment the following line to enable Ebyte module
// #define IS_EBYTE

using cs   = peripherals::ra01::pc13::cs;
using dio0 = peripherals::ra01::pc13::dio0;
using dio1 = peripherals::ra01::pc13::dio1;
using dio3 = peripherals::ra01::pc13::dio3;

using sck  = interfaces::spi4::sck;
using miso = interfaces::spi4::miso;
using mosi = interfaces::spi4::mosi;

#ifdef IS_EBYTE
using txen = Gpio<GPIOE_BASE, 4>;
using rxen = Gpio<GPIOD_BASE, 4>;
#endif

#define SX1278_SPI SPI4

#define SX1278_IRQ_DIO0 EXTI6_IRQHandlerImpl
#define SX1278_IRQ_DIO1 EXTI2_IRQHandlerImpl
#define SX1278_IRQ_DIO3 EXTI11_IRQHandlerImpl

#else
#error "Target not supported"
#endif

SX1278Lora* sx1278 = nullptr;

void __attribute__((used)) SX1278_IRQ_DIO0()
{
    if (sx1278)
        sx1278->handleDioIRQ();
}

void __attribute__((used)) SX1278_IRQ_DIO1()
{
    if (sx1278)
        sx1278->handleDioIRQ();
}

void __attribute__((used)) SX1278_IRQ_DIO3()
{
    if (sx1278)
        sx1278->handleDioIRQ();
}

void initBoard()
{
#ifdef IS_EBYTE
    rxen::mode(Mode::OUTPUT);
    txen::mode(Mode::OUTPUT);
    rxen::low();
    txen::low();
#endif

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
    printf("Received something...\n");

    if (msg.msgid != MAVLINK_MSG_ID_ACK_TM)
    {
        // Prepare ack messages
        mavlink_message_t ackMsg;
        mavlink_msg_ack_tm_pack(1, 1, &ackMsg, msg.msgid, msg.seq);

        // Send the ack back to the sender
        channel->enqueueMsg(ackMsg);
    }
    else
    {
        printf("Received ACK!\n");
    }
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
        printf("Enqueued flight_tm_tm!\n");

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
        printf("Enqueued stats_tm!\n");

        Thread::sleepUntil(start + STATS_TM_PERIOD);
    }
}

int main()
{
    initBoard();

    SX1278Lora::Config config = {};
    config.power              = 15;
    config.ocp                = 0;
    config.coding_rate        = SX1278Lora::Config::Cr::CR_1;
    config.spreading_factor   = SX1278Lora::Config::Sf::SF_7;
    config.bandwidth          = SX1278Lora::Config::Bw::HZ_250000;

    printf("Effective bitrate: %lukb/s\n", config.effectiveBitrate());

    SX1278Lora::Error err;

    SPIBus bus(SX1278_SPI);
    GpioPin cs = cs::getPin();

#ifdef IS_EBYTE
    std::unique_ptr<SX1278::ISX1278Frontend> frontend(
        new EbyteFrontend(txen::getPin(), rxen::getPin()));
#else
    std::unique_ptr<SX1278::ISX1278Frontend> frontend(new RA01Frontend());
#endif

    sx1278 =
        new SX1278Lora(bus, cs, SPI::ClockDivider::DIV_64, std::move(frontend));

    printf("\n[sx1278] Configuring sx1278...\n");
    if ((err = sx1278->init(config)) != SX1278Lora::Error::NONE)
    {
        printf("[sx1278] sx1278->init error: %s\n", "TODO");
        return -1;
    }

    printf("\n[sx1278] Initialization complete!\n");

    channel =
        new Mav(sx1278, &onReceive, SLEEP_AFTER_SEND, MAV_OUT_BUFFER_MAX_AGE);
    channel->start();

    std::thread flight_tm_loop([]() { flightTmLoop(); });
    std::thread stats_tm_loop([]() { statsTmLoop(); });

    while (1)
        Thread::wait();

    return 0;
}
