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
#include <mavlink_lib/gemini/mavlink.h>
// SX1278 includes
#include <radio/SX1278/SX1278Frontends.h>
#include <radio/SX1278/SX1278Fsk.h>
#include <radio/SX1278/SX1278Lora.h>

#include <iostream>
#include <thread>

using namespace miosix;

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
#else
#error "Target not supported"
#endif

// === CONSTANTS ===
static constexpr size_t SX1278_MTU = Boardcore::SX1278Fsk::MTU;
constexpr size_t PACKET_SIZE       = MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM_LEN;

/** @brief Number of packets to send */
constexpr size_t MSG_NUM = 580;

/** @brief End of transmission character */
constexpr uint8_t ACK = 0x06;

static const Boardcore::SX1278Fsk::Config RADIO_CONFIG = {
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

// === GLOBALS ===
Boardcore::SX1278Fsk* sx1278 = nullptr;
Boardcore::SPIBus sx1278_bus(SX1278_SPI);

volatile int dio0_cnt = 0;
volatile int dio1_cnt = 0;
volatile int dio3_cnt = 0;

// === INTERRUPTS ===
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

// === DEFINITIONS ===
void recvLoop();
void sendLoop();
mavlink_payload_flight_tm_t readPacketFromSerial();
void initBoard();

// === MAIN ===
int main()
{
    initBoard();

#if defined SX1278_IS_SENDER
    sendLoop();
#elif defined SX1278_IS_RECEIVER
    recvLoop();
#else
    // this may cause problems with stdin and stdout
    // interfering with each other

    // Actually spawn threads
    std::thread send([]() { sendLoop(); });
    recvLoop();
#endif

    return 0;
}

// === IMPLEMENTATIONS ===
void recvLoop()
{
    uint8_t msg[SX1278_MTU];
    while (1)
    {
        int len = sx1278->receive(msg, sizeof(msg));
        if (len > 0)
        {
            mavlink_payload_flight_tm_t tm;
            memcpy(&tm, msg, PACKET_SIZE);

            // auto serial = miosix::DefaultConsole::instance().get();
            // serial->writeBlock(msg, len, 0);
            std::cout << "[sx1278] Received packet - time: " << tm.timestamp
                      << std::endl;
            // std::cout << "[sx1278] tm.timestamp: " << tm.timestamp <<
            // std::endl; std::cout << "[sx1278] tm.pressure_digi: " <<
            // tm.pressure_digi
            //           << std::endl;
            // std::cout << "[sx1278] tm.pressure_static: " <<
            // tm.pressure_static
            //           << std::endl;
            // std::cout << "[sx1278] tm.airspeed_pitot: " << tm.airspeed_pitot
            //           << std::endl;
            // std::cout << "[sx1278] tm.altitude_agl: " << tm.altitude_agl
            //           << std::endl;
            // std::cout << "[sx1278] tm.acc_x: " << tm.acc_x << std::endl;
            // std::cout << "[sx1278] tm.acc_y: " << tm.acc_y << std::endl;
            // std::cout << "[sx1278] tm.acc_z: " << tm.acc_z << std::endl;
            // std::cout << "[sx1278] tm.gyro_x: " << tm.gyro_x << std::endl;
            // std::cout << "[sx1278] tm.gyro_y: " << tm.gyro_y << std::endl;
            // std::cout << "[sx1278] tm.gyro_z: " << tm.gyro_z << std::endl;
            // std::cout << "[sx1278] tm.mag_x: " << tm.mag_x << std::endl;
            // std::cout << "[sx1278] tm.mag_y: " << tm.mag_y << std::endl;
            // std::cout << "[sx1278] tm.mag_z: " << tm.mag_z << std::endl;
            // std::cout << "[sx1278] tm.gps_lat: " << tm.gps_lat << std::endl;
            // std::cout << "[sx1278] tm.gps_lon: " << tm.gps_lon << std::endl;
            // std::cout << "[sx1278] tm.gps_alt: " << tm.gps_alt << std::endl;
            // std::cout << "[sx1278] tm.left_servo_angle: " <<
            // tm.left_servo_angle
            //           << std::endl;
            // std::cout << "[sx1278] tm.right_servo_angle: "
            //           << tm.right_servo_angle << std::endl;
            // std::cout << "[sx1278] tm.nas_n: " << tm.nas_n << std::endl;
            // std::cout << "[sx1278] tm.nas_e: " << tm.nas_e << std::endl;
            // std::cout << "[sx1278] tm.nas_d: " << tm.nas_d << std::endl;
            // std::cout << "[sx1278] tm.nas_vn: " << tm.nas_vn << std::endl;
            // std::cout << "[sx1278] tm.nas_ve: " << tm.nas_ve << std::endl;
            // std::cout << "[sx1278] tm.nas_vd: " << tm.nas_vd << std::endl;
            // std::cout << "[sx1278] tm.nas_qx: " << tm.nas_qx << std::endl;
            // std::cout << "[sx1278] tm.nas_qy: " << tm.nas_qy << std::endl;
            // std::cout << "[sx1278] tm.nas_qz: " << tm.nas_qz << std::endl;
            // std::cout << "[sx1278] tm.nas_qw: " << tm.nas_qw << std::endl;
            // std::cout << "[sx1278] tm.nas_bias_x: " << tm.nas_bias_x
            //           << std::endl;
            // std::cout << "[sx1278] tm.nas_bias_y: " << tm.nas_bias_y
            //           << std::endl;
            // std::cout << "[sx1278] tm.nas_bias_z: " << tm.nas_bias_z
            //           << std::endl;
            // std::cout << "[sx1278] tm.wes_n: " << tm.wes_n << std::endl;
            // std::cout << "[sx1278] tm.wes_e: " << tm.wes_e << std::endl;
            // std::cout << "[sx1278] tm.vbat: " << tm.vbat << std::endl;
            // std::cout << "[sx1278] tm.vsupply_5v: " << tm.vsupply_5v
            //           << std::endl;
            // std::cout << "[sx1278] tm.temperature: " << tm.temperature
            //           << std::endl;
            // std::cout << "[sx1278] tm.fmm_state: " << tm.fmm_state <<
            // std::endl; std::cout << "[sx1278] tm.nas_state: " << tm.nas_state
            // << std::endl; std::cout << "[sx1278] tm.wes_state: " <<
            // tm.wes_state << std::endl; std::cout << "[sx1278] tm.gps_fix: "
            // << tm.gps_fix << std::endl; std::cout << "[sx1278]
            // tm.pin_nosecone: " << tm.pin_nosecone
            //           << std::endl;
            // std::cout << "[sx1278] tm.logger_error: " << tm.logger_error
            //           << std::endl;
        }
    }
}

void sendLoop()
{
    uint8_t msg[SX1278_MTU];
    while (1)
    {
        mavlink_payload_flight_tm_t tm = readPacketFromSerial();
        // std::cout << "[sx1278] Sending packet" << std::endl;
        memcpy(msg, &tm, PACKET_SIZE);
        sx1278->send(msg, PACKET_SIZE);
    }
}

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
    serial->readBlock(serial_buffer, PACKET_SIZE, 0);
    serial->writeBlock(&ACK, 1, 0);

    // this may be shrunk to the above statement (needs further testing)
    memcpy(ptr_to_tm, serial_buffer, PACKET_SIZE);

    return tm;
}

void initBoard()
{
    printf("[sx1278] Configuring RA01 frontend...\n");
    std::unique_ptr<Boardcore::SX1278::ISX1278Frontend> frontend(
        new Boardcore::RA01Frontend());

    // Run default configuration
    Boardcore::SX1278Fsk::Error err;

    sx1278 = new Boardcore::SX1278Fsk(sx1278_bus, cs::getPin(), dio0::getPin(),
                                      dio1::getPin(), dio3::getPin(),
                                      Boardcore::SPI::ClockDivider::DIV_256,
                                      std::move(frontend));

    printf("\n[sx1278] Configuring sx1278 fsk...\n");
    if ((err = sx1278->init(RADIO_CONFIG)) != Boardcore::SX1278Fsk::Error::NONE)
    {
        // FIXME: Why does clang-format put this line up here?
        printf("[sx1278] sx1278->init error\n");
        return;
    }

    printf("\n[sx1278] Initialization complete!\n");
}
