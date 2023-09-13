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
#include <drivers/timer/TimestampTimer.h>
#include <radio/SX1278/SX1278Frontends.h>
#include <radio/SX1278/SX1278Fsk.h>
#include <scheduler/TaskScheduler.h>
#include <utils/collections/CircularBuffer.h>

#include <random>
#include <thread>

#include "common.h"

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

constexpr uint32_t RADIO_PKT_LENGTH       = SX1278Fsk::MTU;
constexpr uint32_t RADIO_OUT_QUEUE_SIZE   = 20;
constexpr uint32_t RADIO_MAV_MSG_LENGTH   = MAVLINK_MAX_DIALECT_PAYLOAD_SIZE;
constexpr size_t RADIO_OUT_BUFFER_MAX_AGE = 10;
constexpr uint16_t RADIO_SLEEP_AFTER_SEND = 1;

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
#define SX1278_1

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

int msg_queue_idx = 0;
mavlink_message_t msg_queue[10];
FastMutex mutex;

Mav* channel;

void runIdle()
{
    while (1)
        Thread::wait();
}

void fillWithRand(void* data, uint8_t len)
{
    uint8_t* data2 = reinterpret_cast<uint8_t*>(data);
    for (int i = 0; i < len; i++)
    {
        *data2 = rand();
        data2++;
    }
}

void enqueueMsg(const mavlink_message_t& msg)
{
    Lock<FastMutex> lock(mutex);
    if (msg_queue_idx < 10)
    {
        msg_queue[msg_queue_idx] = msg;
        msg_queue_idx++;
    }
}

void sendAck(const mavlink_message_t& msg)
{
    mavlink_message_t ack_msg;
    mavlink_msg_ack_tm_pack(69, 69, &ack_msg, msg.msgid, msg.seq);
    enqueueMsg(ack_msg);
}

mavlink_message_t packStatSysTm()
{
    mavlink_message_t msg;
    mavlink_rocket_stats_tm_t tm = {0};
    fillWithRand(&tm, sizeof(tm));

    mavlink_msg_rocket_stats_tm_encode(69, 69, &msg, &tm);
    return msg;
}

mavlink_message_t packMotorSysTm()
{
    mavlink_message_t msg;
    mavlink_motor_tm_t tm = {0};
    fillWithRand(&tm, sizeof(tm));

    tm.timestamp = TimestampTimer::getTimestamp();

    mavlink_msg_motor_tm_encode(69, 69, &msg, &tm);
    return msg;
}

mavlink_message_t packFlightSysTm()
{
    mavlink_message_t msg;
    mavlink_rocket_flight_tm_t tm = {0};
    fillWithRand(&tm, sizeof(tm));

    tm.timestamp = TimestampTimer::getTimestamp();

    mavlink_msg_rocket_flight_tm_encode(69, 69, &msg, &tm);
    return msg;
}

void onReceive(Mav* channel, const mavlink_message_t& msg)
{
    if (msg.msgid != MAVLINK_MSG_ID_ACK_TM)
    {
        sendAck(msg);
    }
}

void sendPeriodMessages()
{
    {
        Lock<FastMutex> lock(mutex);
        for (int i = 0; i < msg_queue_idx; i++)
        {
            channel->enqueueMsg(msg_queue[i]);
        }

        msg_queue_idx = 0;
    }

    channel->enqueueMsg(packMotorSysTm());
    channel->enqueueMsg(packFlightSysTm());
}

int main()
{
    initBoard();

    SX1278Fsk::Config config = {
        .freq_rf    = 419000000,
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
        runIdle();
    }

    printConfig(config);

    channel = new Mav(sx1278, &onReceive, RADIO_SLEEP_AFTER_SEND,
                      RADIO_OUT_BUFFER_MAX_AGE);
    channel->start();

    Boardcore::TaskScheduler scheduler;

    scheduler.addTask([]() { enqueueMsg(packStatSysTm()); }, 500,
                      TaskScheduler::Policy::RECOVER);
    scheduler.addTask([]() { sendPeriodMessages(); }, 250,
                      TaskScheduler::Policy::RECOVER);

    scheduler.start();

    runIdle();
    return 0;
}
