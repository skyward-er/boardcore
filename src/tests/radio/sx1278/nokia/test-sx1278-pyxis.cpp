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

#include <algorithms/NAS/NASState.h>
#include <drivers/interrupt/external_interrupts.h>
#include <drivers/timer/TimestampTimer.h>
#include <radio/SX1278/SX1278Frontends.h>
#include <radio/SX1278/SX1278Fsk.h>
#include <utils/ButtonHandler/ButtonHandler.h>
#include <utils/collections/CircularBuffer.h>

#include <thread>

#include "../fsk/common.h"
#include "pyxisRocketFlightTm.h"

// Ignore warnings, as we don't want to change third party generated files to
// fix them
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include <mavlink_lib/gemini/mavlink.h>
#pragma GCC diagnostic pop

#include <radio/MavlinkDriver/MavlinkDriver.h>

using namespace Boardcore;
using namespace miosix;

constexpr uint32_t RADIO_PKT_LENGTH     = SX1278Fsk::MTU;
constexpr uint32_t RADIO_OUT_QUEUE_SIZE = 10;
constexpr uint32_t RADIO_MAV_MSG_LENGTH = MAVLINK_MAX_DIALECT_PAYLOAD_SIZE;
constexpr size_t MAV_OUT_BUFFER_MAX_AGE = 10;
constexpr uint32_t FLIGHT_TM_PERIOD     = 250;

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
#define SX1278_IRQ_DIO1 EXTI4_IRQHandlerImpl
#define SX1278_IRQ_DIO3 EXTI11_IRQHandlerImpl

#elif defined _BOARD_STM32F767ZI_GEMINI_GS
#include "interfaces-impl/hwmapping.h"

#define SX1278_IS_SKYWARD433
// #define SX1278_IS_EBYTE

// Comment to use SX1278_2
// #define SX1278_1

#ifdef SX1278_1
using cs   = miosix::radio1::cs;
using dio0 = miosix::radio1::dio0;
using dio1 = miosix::radio1::dio1;
using dio3 = miosix::radio1::dio3;

using sck  = miosix::radio1::spi::sck;
using miso = miosix::radio1::spi::miso;
using mosi = miosix::radio1::spi::mosi;

using txen = miosix::radio1::txen;
using rxen = miosix::radio1::rxen;

#define SX1278_NRST
using rst  = miosix::radio1::nrst;

#define SX1278_SPI MIOSIX_RADIO1_SPI

#define SX1278_IRQ_DIO0 MIOSIX_RADIO1_DIO0_IRQ
#define SX1278_IRQ_DIO1 MIOSIX_RADIO1_DIO1_IRQ
#define SX1278_IRQ_DIO3 MIOSIX_RADIO1_DIO3_IRQ
#else
using cs   = miosix::radio2::cs;
using dio0 = miosix::radio2::dio0;
using dio1 = miosix::radio2::dio1;
using dio3 = miosix::radio2::dio3;

using sck  = miosix::radio2::spi::sck;
using miso = miosix::radio2::spi::miso;
using mosi = miosix::radio2::spi::mosi;

using txen = miosix::radio2::txen;
using rxen = miosix::radio2::rxen;

#define SX1278_NRST
using rst  = miosix::radio2::nrst;

#define SX1278_SPI MIOSIX_RADIO2_SPI

#define SX1278_IRQ_DIO0 MIOSIX_RADIO2_DIO0_IRQ
#define SX1278_IRQ_DIO1 MIOSIX_RADIO2_DIO1_IRQ
#define SX1278_IRQ_DIO3 MIOSIX_RADIO2_DIO3_IRQ
#endif

#else
#error "Target not supported"
#endif

SX1278Fsk* sx1278 = nullptr;

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

void initBoard() {}

struct PendingAck
{
    int msgid;
    int seq;
};

void startBlinking()
{
    std::thread(
        []
        {
            while (1)
            {
                ledOn();
                Thread::sleep(100);
                ledOff();
                Thread::sleep(100);
            }
        })
        .detach();
}

bool launched = false;
CircularBuffer<PendingAck, 10> pending_acks;
FastMutex mutex;

Mav* channel;

void onReceive(Mav* channel, const mavlink_message_t& msg)
{
    if (msg.msgid != MAVLINK_MSG_ID_ACK_TM)
    {
        Lock<FastMutex> l(mutex);
        pending_acks.put({msg.msgid, msg.seq});
    }

    // If command is force launch, trigger the sending of launch telemetry
    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_TC)
    {
        MavCommandList commandId = static_cast<MavCommandList>(
            mavlink_msg_command_tc_get_command_id(&msg));

        if (commandId == MAV_CMD_FORCE_LAUNCH)
        {
            launched = true;
            startBlinking();
        }
    }
}

void flightTmLoop()
{
    int i = 0;

    while (1)
    {
        long long start = miosix::getTick();

        {
            Lock<FastMutex> l(mutex);
            while (!pending_acks.isEmpty())
            {
                auto ack = pending_acks.pop();

                // Prepare ack message
                mavlink_message_t msg;
                mavlink_msg_ack_tm_pack(171, 96, &msg, ack.msgid, ack.seq);

                // Send the ack back to the sender
                channel->enqueueMsg(msg);
            }
        }

        mavlink_message_t msg;
        mavlink_rocket_flight_tm_t tm = {0};

        tm.timestamp  = nasState[i].timestamp;
        tm.nas_n      = nasState[i].n;
        tm.nas_e      = nasState[i].e;
        tm.nas_d      = nasState[i].d;
        tm.nas_vn     = nasState[i].vn;
        tm.nas_ve     = nasState[i].ve;
        tm.nas_vd     = nasState[i].vd;
        tm.nas_qx     = nasState[i].qx;
        tm.nas_qy     = nasState[i].qy;
        tm.nas_qz     = nasState[i].qz;
        tm.nas_qw     = nasState[i].qw;
        tm.nas_bias_x = nasState[i].bx;
        tm.nas_bias_y = nasState[i].by;
        tm.nas_bias_z = nasState[i].bz;

        mavlink_msg_rocket_flight_tm_encode(171, 96, &msg, &tm);

        channel->enqueueMsg(msg);

        Thread::sleepUntil(start + FLIGHT_TM_PERIOD);
        if (launched && i < (sizeof(nasState) / sizeof(nasState[0])) - 1)
        {
            i++;
        }
    }
}

int main()
{
    initBoard();

    SX1278Fsk::Config config = {
        .freq_rf    = 434000000,
        .freq_dev   = 50000,
        .bitrate    = 48000,
        .rx_bw      = Boardcore::SX1278Fsk::Config::RxBw::HZ_125000,
        .afc_bw     = Boardcore::SX1278Fsk::Config::RxBw::HZ_125000,
        .ocp        = 120,
        .power      = 13,
        .shaping    = Boardcore::SX1278Fsk::Config::Shaping::GAUSSIAN_BT_1_0,
        .dc_free    = Boardcore::SX1278Fsk::Config::DcFree::WHITENING,
        .enable_crc = false};

    SX1278Fsk::Error err;

    SPIBus bus(SX1278_SPI);

    std::unique_ptr<SX1278::ISX1278Frontend> frontend(new RA01Frontend());

    sx1278 = new SX1278Fsk(bus, cs::getPin(), dio0::getPin(), dio1::getPin(),
                           dio3::getPin(), SPI::ClockDivider::DIV_64,
                           std::move(frontend));

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

    ButtonHandler::getInstance().registerButtonCallback(
        GpioPin(GPIOA_BASE, 0),
        [](auto event)
        {
            if (event == ButtonEvent::LONG_PRESS)
            {
                launched = true;
                startBlinking();
            }
        });

    flightTmLoop();

    return 0;
}
