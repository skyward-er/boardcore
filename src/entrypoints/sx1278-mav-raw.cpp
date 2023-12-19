/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Davide Mor, Federico Lolli
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

// SX1278 includes
#include <mavlink_lib/gemini/mavlink.h>
#include <radio/SX1278/SX1278Frontends.h>
#include <radio/SX1278/SX1278Fsk.h>
#include <radio/SX1278/SX1278Lora.h>

#include <iostream>
#include <thread>

using namespace miosix;

// Uncomment the following line to enable Lora mode
// Or use SBS to define it for you
// #define SX1278_IS_LORA

#if defined _BOARD_STM32F429ZI_SKYWARD_GS_V2
#include "interfaces-impl/hwmapping.h"

// Uncomment the following line to enable Ebyte module
// #define SX1278_IS_EBYTE
// Uncomment the following line to ebable Skyward433 module
// #define SX1278_IS_SKYWARD433

using cs   = peripherals::ra01::pc13::cs;
using dio0 = peripherals::ra01::pc13::dio0;
using dio1 = peripherals::ra01::pc13::dio1;
using dio3 = peripherals::ra01::pc13::dio3;

using sck  = interfaces::spi4::sck;
using miso = interfaces::spi4::miso;
using mosi = interfaces::spi4::mosi;

#ifdef SX1278_IS_EBYTE
using txen = Gpio<GPIOE_BASE, 4>;
using rxen = Gpio<GPIOD_BASE, 4>;
#endif

#define SX1278_SPI SPI4

#define SX1278_IRQ_DIO0 EXTI6_IRQHandlerImpl
#define SX1278_IRQ_DIO1 EXTI4_IRQHandlerImpl
#define SX1278_IRQ_DIO3 EXTI11_IRQHandlerImpl

#elif defined _BOARD_STM32F429ZI_SKYWARD_RIG
#include "interfaces-impl/hwmapping.h"

#define SX1278_IS_EBYTE

using cs   = radio::cs;
using dio0 = radio::dio0;
using dio1 = radio::dio1;
using dio3 = radio::dio3;

using sck  = radio::sck;
using miso = radio::miso;
using mosi = radio::mosi;

using txen = radio::txEn;
using rxen = radio::rxEn;

#define SX1278_SPI SPI4

#define SX1278_IRQ_DIO0 EXTI5_IRQHandlerImpl
#define SX1278_IRQ_DIO1 EXTI12_IRQHandlerImpl
#define SX1278_IRQ_DIO3 EXTI13_IRQHandlerImpl

#elif defined _BOARD_STM32F767ZI_GEMINI_GS
#include "interfaces-impl/hwmapping.h"
#include "mavlink_lib/gemini/mavlink_msg_payload_flight_tm.h"

// #define SX1278_IS_SKYWARD433
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

#ifdef SX1278_IS_LORA
static constexpr size_t SX1278_MTU = Boardcore::SX1278Lora::MTU;
Boardcore::SX1278Lora* sx1278      = nullptr;
#else
static constexpr size_t SX1278_MTU = Boardcore::SX1278Fsk::MTU;
Boardcore::SX1278Fsk* sx1278       = nullptr;
#endif

volatile int dio0_cnt = 0;
volatile int dio1_cnt = 0;
volatile int dio3_cnt = 0;

#ifdef SX1278_IRQ_DIO0
void __attribute__((used)) SX1278_IRQ_DIO0()
{
    dio0_cnt++;
    if (sx1278)
        sx1278->handleDioIRQ();
}
#endif

#ifdef SX1278_IRQ_DIO1
void __attribute__((used)) SX1278_IRQ_DIO1()
{
    dio1_cnt++;
    if (sx1278)
        sx1278->handleDioIRQ();
}
#endif

#ifdef SX1278_IRQ_DIO3
void __attribute__((used)) SX1278_IRQ_DIO3()
{
    dio3_cnt++;
    if (sx1278)
        sx1278->handleDioIRQ();
}
#endif

void initBoard()
{
#ifdef SX1278_IS_EBYTE
    rxen::mode(Mode::OUTPUT);
    txen::mode(Mode::OUTPUT);
    rxen::low();
    txen::low();
#endif

#ifdef SX1278_NRST
    rst::mode(miosix::Mode::OUTPUT);
    rst::high();
#endif
}

constexpr size_t PACKET_SIZE = 158;

/** @brief Number of packets to send */
constexpr size_t MSG_NUM = 580;

/** @brief End of transmission byte. Used to signal the end of a packet. */
constexpr uint8_t EOT = 0x04;

/**
 * @brief Read a packet from the serial port
 * @warning This function will parse raw bytes coming from
 * serial into the struct
 * @return mavlink_payload_flight_tm_t
 */
mavlink_payload_flight_tm_t readPacketFromSerial()
{
    mavlink_payload_flight_tm_t tm;
    uint8_t* ptr_to_tm = (uint8_t*)&tm;
    uint8_t serial_buffer[PACKET_SIZE];

    auto serial = DefaultConsole::instance().get();
    serial->writeBlock(&EOT, 1, 0);
    serial->readBlock(serial_buffer, PACKET_SIZE, 0);

    // this may be shrunk to the above statement (needs further testing)
    memcpy(ptr_to_tm, serial_buffer, sizeof(mavlink_payload_flight_tm_t));

    return tm;
}

void recvLoop()
{
    uint8_t msg[SX1278_MTU];
    while (1)
    {
        int len = sx1278->receive(msg, sizeof(msg));
        if (len > 0)
        {
            mavlink_payload_flight_tm_t tm;
            memcpy(&tm, msg, sizeof(mavlink_payload_flight_tm_t));
            auto serial = miosix::DefaultConsole::instance().get();
            // serial->writeBlock(msg, len, 0);
            std::cout << "[sx1278] Received packet - time: " << tm.timestamp
                      << std::endl;
        }
    }
}

void sendLoop()
{
    uint8_t msg[SX1278_MTU];
    for (size_t i = 0; i < MSG_NUM; i++)
    {
        mavlink_payload_flight_tm_t tm = readPacketFromSerial();
        std::cout << "[sx1278] Sending packet " << i << std::endl;
        memcpy(msg, &tm, sizeof(mavlink_payload_flight_tm_t));
        sx1278->send(msg, sizeof(mavlink_payload_flight_tm_t));
    }
}

Boardcore::SPIBus sx1278_bus(SX1278_SPI);

int main()
{
    initBoard();

#if defined SX1278_IS_EBYTE
    printf("[sx1278] Confuring Ebyte frontend...\n");
    std::unique_ptr<Boardcore::SX1278::ISX1278Frontend> frontend(
        new Boardcore::EbyteFrontend(txen::getPin(), rxen::getPin()));
#elif defined SX1278_IS_SKYWARD433
    printf("[sx1278] Confuring Skyward 433 frontend...\n");
    std::unique_ptr<Boardcore::SX1278::ISX1278Frontend> frontend(
        new Boardcore::Skyward433Frontend());
#else
    printf("[sx1278] Confuring RA01 frontend...\n");
    std::unique_ptr<Boardcore::SX1278::ISX1278Frontend> frontend(
        new Boardcore::RA01Frontend());
#endif

#ifdef SX1278_IS_LORA
    // Run default configuration
    Boardcore::SX1278Lora::Config config;
    Boardcore::SX1278Lora::Error err;

    sx1278 = new Boardcore::SX1278Lora(sx1278_bus, cs::getPin(), dio0::getPin(),
                                       dio1::getPin(), dio3::getPin(),
                                       Boardcore::SPI::ClockDivider::DIV_64,
                                       std::move(frontend));

    printf("\n[sx1278] Configuring sx1278 lora...\n");
    if ((err = sx1278->init(config)) != Boardcore::SX1278Lora::Error::NONE)
    {
        printf("[sx1278] sx1278->init error\n");
        return -1;
    }

    printf("\n[sx1278] Initialization complete!\n");
#else
    // Run default configuration
    Boardcore::SX1278Fsk::Config config;
    Boardcore::SX1278Fsk::Error err;

    config.freq_rf    = 434000000;
    config.enable_crc = false;

    sx1278 = new Boardcore::SX1278Fsk(sx1278_bus, cs::getPin(), dio0::getPin(),
                                      dio1::getPin(), dio3::getPin(),
                                      Boardcore::SPI::ClockDivider::DIV_256,
                                      std::move(frontend));

    printf("\n[sx1278] Configuring sx1278 fsk...\n");
    if ((err = sx1278->init(config)) != Boardcore::SX1278Fsk::Error::NONE)
    {
        // FIXME: Why does clang-format put this line up here?
        printf("[sx1278] sx1278->init error\n");
        return false;
    }

    printf("\n[sx1278] Initialization complete!\n");
#endif

#if defined SX1278_IS_SENDER
    sendLoop();
#elif defined SX1278_IS_RECEIVER
    recvLoop();
#else
    // Actually spawn threads
    std::thread send([]() { sendLoop(); });
    recvLoop();
#endif

    return 0;
}
