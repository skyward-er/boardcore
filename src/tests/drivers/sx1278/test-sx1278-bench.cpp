/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Davide Mor
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

using namespace Boardcore;
using namespace miosix;

struct Stats;

const char *stringFromErr(SX1278::Error err)
{
    switch (err)
    {
        case SX1278::Error::BAD_VALUE:
            return "Error::BAD_VALUE";

        case SX1278::Error::BAD_VERSION:
            return "Error::BAD_VERSION";

        default:
            return "<unknown>";
    }
}

// Simple xorshift RNG
uint32_t xorshift32()
{
    // https://xkcd.com/221/
    static uint32_t STATE = 0x08104444;

    uint32_t x = STATE;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;

    return STATE = x;
}

#if defined _BOARD_STM32F429ZI_SKYWARD_GS
#include "interfaces-impl/hwmapping.h"

#if 1  // use ra01
using cs   = peripherals::ra01::cs;
using dio0 = peripherals::ra01::dio0;
#else
using cs   = peripherals::sx127x::cs;
using dio0 = peripherals::sx127x::dio0;
#endif

using sck  = interfaces::spi4::sck;
using miso = interfaces::spi4::miso;
using mosi = interfaces::spi4::mosi;

#define SX1278_SPI SPI4

#elif defined _BOARD_STM32F429ZI_SKYWARD_DEATHST_V3
#include "interfaces-impl/hwmapping.h"

using cs   = sensors::sx127x::cs;
using dio0 = sensors::sx127x::dio0;

using sck  = interfaces::spi5::sck;
using miso = interfaces::spi5::miso;
using mosi = interfaces::spi5::mosi;

#define SX1278_SPI SPI5

#else
#error "Target not supported"
#endif

/// Status informations.
struct Stats
{
    int last_recv_packet  = 0;  //< Last received packet ID.
    int corrupted_packets = 0;  //< Packets that got mangled during tx.
    int send_count        = 0;  //< Actual number of packets sent.
    int recv_count        = 0;  //< Actual number of packets received.
    int recv_errors       = 0;  //< Number of failed recvs.

    float packet_loss() const
    {
        if (last_recv_packet != 0)
        {
            return 1.0f - ((float)recv_count / (float)last_recv_packet);
        }
        else
        {
            return 0.0f;
        }
    }

    void print() const
    {
        // Prints are REALLY slow, so take a COPY of stats, so we can print an
        // instant in time.
        Stats stats_now = *this;

        printf("stats.last_recv_packet = %d\n", stats_now.last_recv_packet);
        printf("stats.corrupted_packets = %d\n", stats_now.corrupted_packets);
        printf("stats.send_count = %d\n", stats_now.send_count);
        printf("stats.recv_count = %d\n", stats_now.recv_count);
        printf("stats.recv_errors = %d\n", stats_now.recv_errors);
        printf("stats.packet_loss = %.2f %%\n",
               stats_now.packet_loss() * 100.0f);
    }

} stats;

SX1278 *sx1278 = nullptr;

// Payload size in 32bit blocks
constexpr size_t PAYLOAD_SIZE = 14;

struct Msg
{
    uint32_t idx;
    uint32_t payload[PAYLOAD_SIZE];
};

void recvLoop()
{
    while (1)
    {
        Msg msg;
        memset(&msg, 0, sizeof(msg));

        int len = sx1278->receive((uint8_t *)&msg, sizeof(msg));

        uint32_t acc = 0;
        for (size_t i = 0; i < PAYLOAD_SIZE; i++)
            acc ^= msg.payload[i];

        if (len != sizeof(msg))
        {
            stats.recv_errors++;
        }
        else if (acc != 0)
        {
            stats.recv_errors++;
            stats.corrupted_packets++;
        }
        else
        {
            stats.last_recv_packet = msg.idx;
            stats.recv_count++;
        }
    }
}

void sendLoop()
{
    while (1)
    {
        int next_idx = stats.send_count + 1;

        Msg msg;
        msg.idx = next_idx;

        // Setup the buffer so that the xor of the full thing is 0
        uint32_t acc = 0;
        for (size_t i = 0; i < PAYLOAD_SIZE - 1; i++)
        {
            // Fill only the lower 8bit to simulate a lot of zeroes
            msg.payload[i] = xorshift32() & 0xff;
            acc ^= msg.payload[i];
        }
        msg.payload[PAYLOAD_SIZE - 1] = acc;

        sx1278->send((uint8_t *)&msg, sizeof(msg));
        stats.send_count = next_idx;
    }
}

/// Get current time
long long now() { return miosix::getTick() * 1000 / miosix::TICK_FREQ; }

#if defined _BOARD_STM32F429ZI_SKYWARD_GS
void __attribute__((used)) EXTI6_IRQHandlerImpl()
#elif defined _BOARD_STM32F429ZI_SKYWARD_DEATHST_V3
void __attribute__((used)) EXTI10_IRQHandlerImpl()
#else
#error "Target not supported"
#endif
{
    if (sx1278)
        sx1278->handleDioIRQ();
}

void initBoard()
{
#if defined _BOARD_STM32F429ZI_SKYWARD_GS
    enableExternalInterrupt(GPIOF_BASE, 6, InterruptTrigger::RISING_EDGE);
#elif defined _BOARD_STM32F429ZI_SKYWARD_DEATHST_V3
    enableExternalInterrupt(GPIOF_BASE, 10, InterruptTrigger::RISING_EDGE);
#else
#error "Target not supported"
#endif
}

int main()
{
    initBoard();

    // Run default configuration
    SX1278::Config config;
    config.freq_rf = 430000000;

    SX1278::Error err;

    SPIBus bus(SX1278_SPI);
    GpioPin cs = cs::getPin();

    sx1278 = new SX1278(bus, cs);

    printf("\n[sx1278] Configuring sx1278...\n");
    if ((err = sx1278->init(config)) != SX1278::Error::NONE)
    {
        printf("[sx1278] sx1278->init error: %s\n", stringFromErr(err));
        return -1;
    }

    // Run background threads
#ifndef DISABLE_RX
    std::thread recv([]() { recvLoop(); });
#endif
#ifndef DISABLE_TX
    std::thread send([]() { sendLoop(); });
#endif

    // Finish!
    long long start = now();

    printf("\n[sx1278] Initialization complete!\n");

    miosix::Thread::sleep(4000);

    while (1)
    {
        long long elapsed = now() - start;

        // Calculate bitrates
        float tx_bitrate = (float)(stats.send_count * sizeof(Msg) * 8) /
                           ((float)elapsed / 1000.0f);

        float rx_bitrate = (float)(stats.recv_count * sizeof(Msg) * 8) /
                           ((float)elapsed / 1000.0f);

        printf("\n[sx1278] Stats:\n");
        stats.print();
        printf("tx_bitrate: %.2f kb/s\n", tx_bitrate / 1000.0f);
        printf("rx_bitrate: %.2f kb/s\n", rx_bitrate / 1000.0f);

        miosix::Thread::sleep(2000);
    }

    return 0;
}
