 /* Copyright (c) 2026 Skyward Experimental Rocketry
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
#include <filesystem/console/console_device.h>
#include <radio/SX127X/SX127X.h>
#include <radio/SX127X/SX127XFrontends.h>

#include <array>
#include <cstdio>
#include <memory>
#include <thread>

using namespace miosix;

#if !defined(_BOARD_STM32F767ZI_GEMINI_GS)
#error "Target not supported"
#endif

#include "interfaces-impl/hwmapping.h"


using cs   = Gpio<GPIOG_BASE, 7>;
using dio0 = Gpio<GPIOE_BASE, 3>;
using dio1 = Gpio<GPIOE_BASE, 4>;
using dio3 = Gpio<GPIOA_BASE, 8>;
using rst  = Gpio<GPIOA_BASE, 4>;

#define SX127X_SPI SPI4
#define SX127X_IRQ_DIO0 EXTI3_IRQHandlerImpl
#define SX127X_IRQ_DIO1 EXTI4_IRQHandlerImpl
#define SX127X_IRQ_DIO3 EXTI8_IRQHandlerImpl

static constexpr const char* SX127X_RADIO_SLOT = "radio2";


using PaEnable  = miosix::Gpio<GPIOG_BASE, 12>;
using LnaEnable = miosix::Gpio<GPIOB_BASE, 7>;

namespace
{

enum class TestPin
{
    PA_ENABLE,
    LNA_ENABLE
};

static constexpr uint32_t RADIO_FREQ_HZ       = 868000000UL;
static constexpr int8_t DEFAULT_TX_POWER_DBM  = 15;
static constexpr int8_t TX_POWER_MIN_DBM      = 0;
static constexpr int8_t TX_POWER_MAX_DBM      = 15;
static constexpr size_t SX127X_PACKET_MTU     = 255;
static constexpr size_t MAX_COMMAND_LENGTH    = SX127X_PACKET_MTU + 16;
static constexpr size_t MAX_COMMAND_TOKENS    = 4;
static constexpr unsigned int RX_POLL_SLEEP_MS = 5;

std::unique_ptr<Boardcore::SX127X> radio;
bool radioRunning        = false;
int8_t currentTxPowerDbm = DEFAULT_TX_POWER_DBM;
bool paEnableHigh        = false;
bool lnaEnableHigh       = false;


const char* testPinName(TestPin pin)
{
    return pin == TestPin::PA_ENABLE ? "PA_ENABLE" : "LNA_ENABLE";
}

const char* testPinGpioName(TestPin pin)
{
    return pin == TestPin::PA_ENABLE ? "GPIOG12" : "GPIOB7";
}




template <typename Pin>
void writePinLevel(bool high)
{
    if (high)
        Pin::high();
    else
        Pin::low();
}

void initTestPins()
{
    PaEnable::mode(Mode::OUTPUT);
    LnaEnable::mode(Mode::OUTPUT);
    writePinLevel<PaEnable>(false);
    writePinLevel<LnaEnable>(false);
    paEnableHigh = false;
    lnaEnableHigh = false;
    PaEnable::low();
    LnaEnable::low();
}

void setTestPinLevel(TestPin pin, bool high)
{
    if (pin == TestPin::PA_ENABLE)
    {
        writePinLevel<PaEnable>(high);
        paEnableHigh = high;
        return;
    }

    writePinLevel<LnaEnable>(high);
    lnaEnableHigh = high;
}


void printReceivedPacket(const uint8_t* data, size_t len)
{
    auto serial = DefaultConsole::instance().get();

    static constexpr char prefix[] = "RECV -> ";
    static constexpr char suffix[] = "\n";

    serial->writeBlock(prefix, sizeof(prefix) - 1, 0);

    for (size_t i = 0; i < len; i++)
    {
        char ch = std::isprint(data[i]) ? static_cast<char>(data[i]) : '.';
        serial->writeBlock(&ch, 1, 0);
    }

    serial->writeBlock(suffix, sizeof(suffix) - 1, 0);
}

void recvLoop()
{
    std::array<uint8_t, SX127X_PACKET_MTU> packet{};

    while (true)
    {
        if (!radio)
        {
            Thread::sleep(RX_POLL_SLEEP_MS);
            continue;
        }

        const ssize_t len = radio->dequeuePacket(packet.data(), packet.size());
        if (len > 0)
        {
            printReceivedPacket(packet.data(), static_cast<size_t>(len));
        }
        else
        {
            Thread::sleep(RX_POLL_SLEEP_MS);
        }
    }
}


void txLoop()
{
    static constexpr const char payload[] =
        "Sucapalle Dio0! 0!Sucapalle Dio0!Sucapalle Dio0!Sucapalle Dio0!Sucapalle Dio0!";

    while (true)
    {
        Thread::sleep(500);

        if (!radio || !radioRunning)
        {
            printf("radio non pronta\n");
            continue;
        }

        const bool ok = radio->enqueuePacket(
            reinterpret_cast<const uint8_t*>(payload),
            sizeof(payload) - 1
        );

        if (!ok)
        {
            printf("invio non riuscito\n");
            continue;
        }

        printf("TX -> %s\n", payload);
    }
}
Boardcore::SX127X::Config buildRadioConfig()
{
    Boardcore::SX127XFsk::Config phy;
    phy.freq_rf    = RADIO_FREQ_HZ;
    phy.power      = currentTxPowerDbm;
    phy.enable_crc = false;

    return Boardcore::SX127X::FskConfig{phy};
}

void initBoard()
{
    cs::mode(Mode::OUTPUT);
    cs::high();
    dio0::mode(Mode::INPUT_PULL_UP);
    dio1::mode(Mode::INPUT_PULL_UP);
    dio3::mode(Mode::INPUT_PULL_UP);
    rst::mode(Mode::OUTPUT);
    rst::high();
    initTestPins();
}

}  // namespace

#ifdef SX127X_IRQ_DIO0
void __attribute__((used)) SX127X_IRQ_DIO0()
{
    if (radio)
        radio->handleDioIRQ();
}
#endif

#ifdef SX127X_IRQ_DIO1
void __attribute__((used)) SX127X_IRQ_DIO1()
{
    if (radio)
        radio->handleDioIRQ();
}
#endif

#ifdef SX127X_IRQ_DIO3
void __attribute__((used)) SX127X_IRQ_DIO3()
{
    if (radio)
        radio->handleDioIRQ();
}
#endif

Boardcore::SPIBus sx127xBus(SX127X_SPI);

int main()
{
    initBoard();

    Boardcore::SX127X::Buffers buffers{};
    Boardcore::SX127X::Timeouts timeouts{};

    radio = std::make_unique<Boardcore::SX127X>(
        sx127xBus, cs::getPin(), dio0::getPin(), dio1::getPin(),
        dio3::getPin(), Boardcore::SPI::ClockDivider::DIV_256,
        std::make_unique<Boardcore::Skyward433Frontend>(),
        buildRadioConfig(), buffers, timeouts, SX127X_PACKET_MTU);

    if (!radio->start())
    {
        printf("avvio fallito\n");
        return -1;
    }

    radioRunning = true;

    std::thread recv([]() { recvLoop(); });
    (void)recv;

    printf("SX127X FSK  %s\n", SX127X_RADIO_SLOT);
    printf("TX loop ogni 500 ms avviato\n");
    txLoop();

    return 0;
}
