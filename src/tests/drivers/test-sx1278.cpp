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

#include <cstring>
#include <thread>

using namespace Boardcore;
using namespace miosix;

/**
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

SPIBus bus1(SPI3);
SPIBus bus2(SPI2);

GpioPin sck1(GPIOC_BASE, 10);
GpioPin miso1(GPIOC_BASE, 11);
GpioPin mosi1(GPIOC_BASE, 12);
GpioPin cs1(GPIOA_BASE, 1);
GpioPin dio1(GPIOC_BASE, 15);

GpioPin sck2(GPIOB_BASE, 13);
GpioPin miso2(GPIOB_BASE, 14);
GpioPin mosi2(GPIOB_BASE, 15);
GpioPin cs2(GPIOA_BASE, 2);
GpioPin dio2(GPIOC_BASE, 0);

const char *stringFromBool(bool b)
{
    if (b)
    {
        return "true";
    }
    else
    {
        return "false";
    }
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

/// Status informations.
struct Stats
{
    int last_sent_packet  = 0;  //< Last sent packet ID.
    int last_recv_packet  = 0;  //< Last received packet ID.
    int corrupted_packets = 0;  //< Packets that got mangled during tx.
    int recv_count        = 0;  //< Actual number of packets received.
    int recv_errors       = 0;  //< Number of failed recvs.

    float packet_loss() const
    {
        return 1.0f - ((float)recv_count / (float)last_recv_packet);
    }

} stats;

void printConfig(SX1278::Config config)
{
    printf("config.freq_rf = %d\n", config.freq_rf);
    printf("config.freq_dev = %d\n", config.freq_dev);
    printf("config.bitrate = %d\n", config.bitrate);
    printf("config.rx_bw = %s\n", stringFromRxBw(config.rx_bw));
    printf("config.afc_bw = %s\n", stringFromRxBw(config.afc_bw));
    printf("config.ocp = %d\n", config.ocp);
    printf("config.enable_int = %s\n", stringFromBool(config.enable_int));
    printf("config.power = %d\n", config.power);
}

void printStats(Stats stats)
{
    // Prints are REALLY slow, so take a COPY of stats, so we can print an
    // instant in time.

    printf("stats.last_sent_packet = %d\n", stats.last_sent_packet);
    printf("stats.last_recv_packet = %d\n", stats.last_recv_packet);
    printf("stats.corrupted_packets = %d\n", stats.corrupted_packets);
    printf("stats.recv_count = %d\n", stats.recv_count);
    printf("stats.recv_errors = %d\n", stats.recv_errors);
    printf("stats.packet_loss = %.2f %%\n", stats.packet_loss() * 100.0f);
}

/// Interval between transmissions.
const int TX_INTERVAL = 1;

SX1278 *sx1278_rx = nullptr;
SX1278 *sx1278_tx = nullptr;

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

void __attribute__((used)) EXTI15_IRQHandlerImpl()
{
    if (sx1278_rx)
        sx1278_rx->handleDioIRQ();
}

void __attribute__((used)) EXTI0_IRQHandlerImpl()
{
    if (sx1278_tx)
        sx1278_tx->handleDioIRQ();
}

void recvLoop()
{
    while (1)
    {
        Msg msg;
        msg.idx     = 0;
        msg.dummy_1 = 0;
        msg.dummy_2 = 0;
        msg.dummy_3 = 0;

        int len = sx1278_rx->recv((uint8_t *)&msg, sizeof(msg));
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

void sendLoop()
{
    while (1)
    {
        int next_idx = stats.last_sent_packet + 1;

        Msg msg;
        msg.idx     = next_idx;
        msg.dummy_1 = Msg::DUMMY_1;
        msg.dummy_2 = Msg::DUMMY_2;
        msg.dummy_3 = Msg::DUMMY_3;

        sx1278_tx->send((uint8_t *)&msg, sizeof(msg));
        stats.last_sent_packet = next_idx;

        miosix::Thread::sleep(TX_INTERVAL);
    }
}

/// Get current time
long long now() { return miosix::getTick() * 1000 / miosix::TICK_FREQ; }

/// Initialize stm32f407g board
void initBoard()
{
    {
        miosix::FastInterruptDisableLock dLock;

        // Enable SPI1 and SPI2
        RCC->APB1ENR |= RCC_APB1ENR_SPI2EN | RCC_APB1ENR_SPI3EN;
        RCC_SYNC();

        // Setup SPI pins
        sck1.mode(miosix::Mode::ALTERNATE);
        sck1.alternateFunction(6);
        miso1.mode(miosix::Mode::ALTERNATE);
        miso1.alternateFunction(6);
        mosi1.mode(miosix::Mode::ALTERNATE);
        mosi1.alternateFunction(6);

        sck2.mode(miosix::Mode::ALTERNATE);
        sck2.alternateFunction(5);
        miso2.mode(miosix::Mode::ALTERNATE);
        miso2.alternateFunction(5);
        mosi2.mode(miosix::Mode::ALTERNATE);
        mosi2.alternateFunction(5);

        cs1.mode(miosix::Mode::OUTPUT);
        dio1.mode(miosix::Mode::INPUT);

        cs2.mode(miosix::Mode::OUTPUT);
        dio2.mode(miosix::Mode::INPUT);
    }

    cs1.high();
    cs2.high();

    enableExternalInterrupt(GPIOC_BASE, 0, InterruptTrigger::RISING_EDGE);
    enableExternalInterrupt(GPIOC_BASE, 15, InterruptTrigger::RISING_EDGE);
}

int main()
{
    initBoard();

    sx1278_rx = new SX1278(bus1, cs1);
    sx1278_tx = new SX1278(bus2, cs2);

    // Run default configuration
    SX1278::Config config_rx;
    config_rx.enable_int = true;

    SX1278::Config config_tx;
    config_tx.enable_int = true;

    SX1278::Error err;

    printf("\n[sx1278] Configuring sx1278_rx...\n");
    printConfig(config_rx);
    if ((err = sx1278_rx->init(config_rx)) != SX1278::Error::NONE)
    {
        printf("[sx1278] sx1278_rx->init error: %s\n", stringFromErr(err));
        return -1;
    }

    printf("\n[sx1278] Configuring sx1278_tx...\n");
    printConfig(config_tx);
    if ((err = sx1278_tx->init(config_tx)) != SX1278::Error::NONE)
    {
        printf("[sx1278] sx1278_tx->init error: %s\n", stringFromErr(err));
        return -1;
    }

    printf("\n[sx1278] Initialization complete!\n");

    std::thread recv(&recvLoop);
    std::thread send(&sendLoop);
    long long start = now();

    miosix::Thread::sleep(4000);

    while (1)
    {
        printf("\n[sx1278] Stats:\n");
        printStats(stats);

        long long elapsed = now() - start;

        // Calculate effective bitrate
        float eff_bitrate = (float)(stats.recv_count * sizeof(Msg) * 8) /
                            ((float)elapsed / 1000.0f);
        printf("Effective bitrate: %.2f kb/s\n", eff_bitrate / 1000.0f);

        miosix::Thread::sleep(2000);
    }

    return 0;
}
