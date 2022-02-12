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

#include <drivers/SX1278/SX1278.h>
#include <drivers/interrupt/external_interrupts.h>

#include <thread>

using namespace Boardcore;
using namespace miosix;

/**
 * 0 -> RX
 * 1 -> TX
 *
 * Connection diagram:
 * sx1278[0]:nss  -> stm32:pa1
 * sx1278[0]:dio0 -> stm32:pc15
 * sx1278[0]:mosi -> stm32:pc12 (SPI3_MOSI)
 * sx1278[0]:miso -> stm32:pc11 (SPI3_MISO)
 * sx1278[0]:sck  -> stm32:pc10 (SPI3_SCK)

 * sx1278[1]:nss  -> stm32:pa2
 * sx1278[1]:dio0 -> stm32:pc0
 * sx1278[1]:mosi -> stm32:pb15 (SPI2_MOSI)
 * sx1278[1]:miso -> stm32:pb14 (SPI2_MISO)
 * sx1278[1]:sck  -> stm32:pb13 (SPI2_SCK)
 */

SPIBus bus0(SPI3);
SPIBus bus1(SPI2);

GpioPin sck0(GPIOC_BASE, 10);
GpioPin miso0(GPIOC_BASE, 11);
GpioPin mosi0(GPIOC_BASE, 12);
GpioPin cs0(GPIOA_BASE, 1);
GpioPin dio0(GPIOC_BASE, 15);

GpioPin sck1(GPIOB_BASE, 13);
GpioPin miso1(GPIOB_BASE, 14);
GpioPin mosi1(GPIOB_BASE, 15);
GpioPin cs1(GPIOA_BASE, 2);
GpioPin dio1(GPIOC_BASE, 0);

struct Stats;

const char *stringFromErr(SX1278::Error err);
const char *stringFromRxBw(SX1278::RxBw rx_bw);

void printStats(Stats stats);
void printConfig(SX1278::Config config);

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

} stats;

/// Interval between transmissions.
const int TX_INTERVAL = 1;

SX1278 *sx1278[2] = {nullptr, nullptr};

struct Msg
{
    int idx;
    int dummy_1;
    int dummy_2;
    int dummy_3;

    const static int DUMMY_1 = 0xdeadbeef;
    const static int DUMMY_2 = 0xbeefdead;
    const static int DUMMY_3 = 0x1234abcd;
};

void recvLoop(int idx)
{
    while (1)
    {
        Msg msg;
        msg.idx     = 0;
        msg.dummy_1 = 0;
        msg.dummy_2 = 0;
        msg.dummy_3 = 0;

        int len = sx1278[idx]->receive((uint8_t *)&msg, sizeof(msg));
        if (len != sizeof(msg))
        {
            stats.recv_errors++;
        }
        else if (msg.dummy_1 != Msg::DUMMY_1 || msg.dummy_2 != Msg::DUMMY_2 ||
                 msg.dummy_3 != Msg::DUMMY_3)
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

void sendLoop(int idx)
{
    while (1)
    {
        miosix::Thread::sleep(TX_INTERVAL);

        int next_idx = stats.send_count + 1;

        Msg msg;
        msg.idx     = next_idx;
        msg.dummy_1 = Msg::DUMMY_1;
        msg.dummy_2 = Msg::DUMMY_2;
        msg.dummy_3 = Msg::DUMMY_3;

        sx1278[idx]->send((uint8_t *)&msg, sizeof(msg));
        stats.send_count = next_idx;
    }
}

/// Get current time
long long now() { return miosix::getTick() * 1000 / miosix::TICK_FREQ; }

void __attribute__((used)) EXTI15_IRQHandlerImpl()
{
    if (sx1278[0])
        sx1278[0]->handleDioIRQ();
}

void __attribute__((used)) EXTI0_IRQHandlerImpl()
{
    if (sx1278[1])
        sx1278[1]->handleDioIRQ();
}

/// Initialize stm32f407g board (sx1278[0] only)
void initBoard0()
{
    {
        miosix::FastInterruptDisableLock dLock;

        // Enable SPI3
        RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
        RCC_SYNC();

        // Setup SPI pins
        sck0.mode(miosix::Mode::ALTERNATE);
        sck0.alternateFunction(6);
        miso0.mode(miosix::Mode::ALTERNATE);
        miso0.alternateFunction(6);
        mosi0.mode(miosix::Mode::ALTERNATE);
        mosi0.alternateFunction(6);

        cs0.mode(miosix::Mode::OUTPUT);
        dio0.mode(miosix::Mode::INPUT);
    }

    cs0.high();
    enableExternalInterrupt(dio0.getPort(), dio0.getNumber(),
                            InterruptTrigger::RISING_EDGE);
}

/// Initialize stm32f407g board (sx1278[1] only)
void initBoard1()
{
    {
        miosix::FastInterruptDisableLock dLock;

        // Enable SPI2
        RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
        RCC_SYNC();

        sck1.mode(miosix::Mode::ALTERNATE);
        sck1.alternateFunction(5);
        miso1.mode(miosix::Mode::ALTERNATE);
        miso1.alternateFunction(5);
        mosi1.mode(miosix::Mode::ALTERNATE);
        mosi1.alternateFunction(5);

        cs1.mode(miosix::Mode::OUTPUT);
        dio1.mode(miosix::Mode::INPUT);
    }

    cs1.high();
    enableExternalInterrupt(dio1.getPort(), dio1.getNumber(),
                            InterruptTrigger::RISING_EDGE);
}

int main()
{
#ifndef DISABLE_RX
    initBoard0();
#endif
#ifndef DISABLE_TX
    initBoard1();
#endif

    // Run default configuration
    SX1278::Config config;
    SX1278::Error err;

    // Configure them
#ifndef DISABLE_RX
    sx1278[0] = new SX1278(bus0, cs0);

    printf("\n[sx1278] Configuring sx1278[0]...\n");
    printConfig(config);
    if ((err = sx1278[0]->init(config)) != SX1278::Error::NONE)
    {
        printf("[sx1278] sx1278[0]->init error: %s\n", stringFromErr(err));
        return -1;
    }
#endif

#ifndef DISABLE_TX
    sx1278[1] = new SX1278(bus1, cs1);

    printf("\n[sx1278] Configuring sx1278[1]...\n");
    printConfig(config);
    if ((err = sx1278[1]->init(config)) != SX1278::Error::NONE)
    {
        printf("[sx1278] sx1278[1]->init error: %s\n", stringFromErr(err));
        return -1;
    }
#endif

    // Run background threads
#ifndef DISABLE_RX
    std::thread recv([]() { recvLoop(0); });
#endif
#ifndef DISABLE_TX
    std::thread send([]() { sendLoop(1); });
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
        printStats(stats);
        printf("tx_bitrate: %.2f kb/s\n", tx_bitrate / 1000.0f);
        printf("rx_bitrate: %.2f kb/s\n", rx_bitrate / 1000.0f);

        miosix::Thread::sleep(2000);
    }

    return 0;
}

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

const char *stringFromRxBw(SX1278::RxBw rx_bw)
{
    switch (rx_bw)
    {
        case SX1278::RxBw::HZ_2600:
            return "RxBw::HZ_2600";

        case SX1278::RxBw::HZ_3100:
            return "RxBw::HZ_3100";

        case SX1278::RxBw::HZ_3900:
            return "RxBw::HZ_3900";

        case SX1278::RxBw::HZ_5200:
            return "RxBw::HZ_5200";

        case SX1278::RxBw::HZ_6300:
            return "RxBw::HZ_6300";

        case SX1278::RxBw::HZ_7800:
            return "RxBw::HZ_7800";

        case SX1278::RxBw::HZ_10400:
            return "RxBw::HZ_10400";

        case SX1278::RxBw::HZ_12500:
            return "RxBw::HZ_12500";

        case SX1278::RxBw::HZ_15600:
            return "RxBw::HZ_15600";

        case SX1278::RxBw::HZ_20800:
            return "RxBw::HZ_20800";

        case SX1278::RxBw::HZ_25000:
            return "RxBw::HZ_25000";

        case SX1278::RxBw::HZ_31300:
            return "RxBw::HZ_31300";

        case SX1278::RxBw::HZ_41700:
            return "RxBw::HZ_41700";

        case SX1278::RxBw::HZ_50000:
            return "RxBw::HZ_50000";

        case SX1278::RxBw::HZ_62500:
            return "RxBw::HZ_62500";

        case SX1278::RxBw::HZ_83300:
            return "RxBw::HZ_83300";

        case SX1278::RxBw::HZ_100000:
            return "RxBw::HZ_100000";

        case SX1278::RxBw::HZ_125000:
            return "RxBw::HZ_125000";

        case SX1278::RxBw::HZ_166700:
            return "RxBw::HZ_166700";

        case SX1278::RxBw::HZ_200000:
            return "RxBw::HZ_200000";

        case SX1278::RxBw::HZ_250000:
            return "RxBw::HZ_250000";

        default:
            return "<unknown>";
    }
}

void printConfig(SX1278::Config config)
{
    printf("config.freq_rf = %d\n", config.freq_rf);
    printf("config.freq_dev = %d\n", config.freq_dev);
    printf("config.bitrate = %d\n", config.bitrate);
    printf("config.rx_bw = %s\n", stringFromRxBw(config.rx_bw));
    printf("config.afc_bw = %s\n", stringFromRxBw(config.afc_bw));
    printf("config.ocp = %d\n", config.ocp);
    printf("config.power = %d\n", config.power);
}

void printStats(Stats stats)
{
    // Prints are REALLY slow, so take a COPY of stats, so we can print an
    // instant in time.

    printf("stats.last_recv_packet = %d\n", stats.last_recv_packet);
    printf("stats.corrupted_packets = %d\n", stats.corrupted_packets);
    printf("stats.send_count = %d\n", stats.send_count);
    printf("stats.recv_count = %d\n", stats.recv_count);
    printf("stats.recv_errors = %d\n", stats.recv_errors);
    printf("stats.packet_loss = %.2f %%\n", stats.packet_loss() * 100.0f);
}
