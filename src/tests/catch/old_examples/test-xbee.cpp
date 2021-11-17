/* Copyright (c) 2019 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

#ifdef STANDALONE_CATCH1_TEST
#include "catch-tests-entry.cpp"
#endif

#include <catch2/catch.hpp>
#include <vector>

#define private public

#include "drivers/Xbee/Xbee.h"
#include "utils/testutils/BusTemplateMock.h"

using std::vector;

// No-op GPIO mock
class GpioMock
{
public:
    static int value() { return 0; }

    static void high() { (void)0; }

    static void low() { (void)0; }
};

typedef Xbee::Xbee<BusSPIMock, GpioMock, GpioMock> xbee_t;

class XbeeTestFixture
{
public:
    XbeeTestFixture() {}

    ~XbeeTestFixture() { BusSPIMock::getInstance()->restoreState(); }

protected:
    BusSPIMock& spi = *BusSPIMock::getInstance();
    xbee_t xbee;
};

void buildRxPacket(vector<uint8_t>& packet, const vector<uint8_t>& payload)
{
    using namespace Xbee;
    packet.reserve(API_HEADER_SIZE + RX_FRAME_HEADER_SIZE + payload.size() + 1);

    packet.push_back(START_DELIMITER);
    uint16_t frame_size = RX_FRAME_HEADER_SIZE + payload.size();
    packet.push_back((frame_size & 0xFF00) >> 8);
    packet.push_back(frame_size & 0xFF);

    packet.push_back(xbee_t::FRAMETYPE_RX_PACKET);
    uint64_t address = 0xFFFF;

    for (int i = 7; i >= 0; i--)
    {
        packet.push_back((address >> i * 8) & 0xFF);
    }

    packet.push_back(0xFF);
    packet.push_back(0xFE);
    packet.push_back(0x40);

    packet.insert(packet.end(), payload.begin(), payload.end());

    uint8_t checksum = 0;
    for (auto it = packet.begin() + 3; it != packet.end(); it++)
    {
        checksum += *it;
    }
    packet.push_back(0xFF - checksum);
}

TEST_CASE_METHOD(XbeeTestFixture, "[Xbee] Receive with transferData()")
{
    SECTION("Test single receive")
    {
        vector<uint8_t> packet, payload;
        payload.resize(25, 0x55);
        buildRxPacket(packet, payload);

        REQUIRE(spi.getMOSI().size() == 0);

        spi.addMISO(packet);

        xbee.transferData();

        REQUIRE(spi.getMOSI().size() == packet.size());

        vector<uint8_t> xbee_payload;

        xbee_payload.insert(xbee_payload.end(), xbee.rx_frame.begin(),
                            xbee.rx_frame.end());

        REQUIRE(xbee_payload == payload);
    }

  /*  SECTION("Test single receive with no payload")
    {
        vector<uint8_t> packet, payload;
        buildRxPacket(packet, payload);

        REQUIRE(spi.getMOSI().size() == 0);

        spi.addMISO(packet);

        xbee.transferData();

        REQUIRE(spi.getMOSI().size() == packet.size());

        REQUIRE(xbee.rx_frame.size() == 0);
    }

    SECTION("Test double consecutive receive")
    {
        vector<uint8_t> packet, payload;
        payload.resize(25, 0x55);
        buildRxPacket(packet, payload);

        REQUIRE(spi.getMOSI().size() == 0);

        spi.addMISO(packet);
        spi.addMISO(packet);

        xbee.transferData();
        xbee.transferData();

        REQUIRE(spi.getMOSI().size() == packet.size() * 2);

        vector<uint8_t> xbee_payload;

        xbee_payload.insert(xbee_payload.end(), xbee.rx_frame.begin(),
                            xbee.rx_frame.end());

        REQUIRE(xbee_payload == payload);
    }
}

TEST_CASE_METHOD(XbeeTestFixture, "[Xbee] Full duplex with transferData()")
{
    SECTION("Simple transmit")
    {
        vector<uint8_t> out(20, 0x55);
        REQUIRE(xbee.tx_buf.size() == 0);

        xbee.setTxBuf(out.data(), out.size());
        REQUIRE(xbee.tx_buf.size() == out.size());

        xbee.transferData();
        REQUIRE(xbee.tx_buf.size() == 0);

        REQUIRE(spi.getMOSI() == out);
    }

    SECTION("Transmit + Smaller receive starting at the same time")
    {
        vector<uint8_t> out(40, 0x44);
        xbee.setTxBuf(out.data(), out.size());

        vector<uint8_t> packet, payload;
        payload.resize(5, 0x55);
        buildRxPacket(packet, payload);

        spi.addMISO(packet);

        xbee.transferData();

        REQUIRE(spi.getMOSI() == out);

        vector<uint8_t> xbee_payload;

        xbee_payload.insert(xbee_payload.end(), xbee.rx_frame.begin(),
                            xbee.rx_frame.end());

        REQUIRE(xbee_payload == payload);
    }

    SECTION("Transmit + 2 receives")
    {
        vector<uint8_t> out(40, 0x44);
        xbee.setTxBuf(out.data(), out.size());

        vector<uint8_t> packet, payload;
        payload.resize(5, 0x55);
        buildRxPacket(packet, payload);

        spi.addMISO(packet);

        xbee.transferData();

        REQUIRE(spi.getMOSI() == out);

        vector<uint8_t> xbee_payload;

        xbee_payload.insert(xbee_payload.end(), xbee.rx_frame.begin(),
                            xbee.rx_frame.end());

        REQUIRE(xbee_payload == payload);

        // Receive 2

        // Expcted output
        spi.restoreState();
        xbee_payload.clear();

        vector<uint8_t> expectedMOSI(packet.size(), 0x00);

        spi.addMISO(packet);

        xbee.transferData();

        REQUIRE(spi.getMOSI() == expectedMOSI);

        xbee_payload.insert(xbee_payload.end(), xbee.rx_frame.begin(),
                            xbee.rx_frame.end());

        REQUIRE(xbee_payload == payload);
    }

    SECTION("Transmit + Bigger receive starting at the same time")
    {
        vector<uint8_t> out(20, 0x44);
        xbee.setTxBuf(out.data(), out.size());

        vector<uint8_t> packet, payload;
        payload.resize(20, 0x55);
        buildRxPacket(packet, payload);

        spi.addMISO(packet);

        xbee.transferData();

        out.resize(packet.size(), 0);
        REQUIRE(spi.getMOSI() == out);

        vector<uint8_t> xbee_payload;

        xbee_payload.insert(xbee_payload.end(), xbee.rx_frame.begin(),
                            xbee.rx_frame.end());

        REQUIRE(xbee_payload == payload);
    }

    SECTION("Transmit + Bigger receive starting later")
    {
        vector<uint8_t> out(20, 0x44);
        xbee.setTxBuf(out.data(), out.size());

        vector<uint8_t> packet, payload;
        payload.resize(20, 0x55);
        buildRxPacket(packet, payload);
        packet.insert(packet.begin(), 10, 0x00);

        spi.addMISO(packet);

        xbee.transferData();

        out.resize(packet.size(), 0);
        REQUIRE(spi.getMOSI() == out);

        vector<uint8_t> xbee_payload;

        xbee_payload.insert(xbee_payload.end(), xbee.rx_frame.begin(),
                            xbee.rx_frame.end());

        REQUIRE(xbee_payload == payload);
    }

    SECTION("Transmit + Receive starting at incrementing bytes later")
    {
        for (int i = 0; i <= 20; i++)
        {
            vector<uint8_t> out(20, 0x44);
            xbee.setTxBuf(out.data(), out.size());

            vector<uint8_t> packet, payload;
            payload.resize(2, 0x55);

            buildRxPacket(packet, payload);

            packet.insert(packet.begin(), i, 0x00);
            spi.addMISO(packet);

            xbee.transferData();
            if(i == 20)
            {
                 xbee.transferData();
            }

            if (out.size() < packet.size())
            {
                out.resize(packet.size(), 0x00);
            }

            REQUIRE(spi.getMOSI() == out);

            vector<uint8_t> xbee_payload;

            xbee_payload.insert(xbee_payload.end(), xbee.rx_frame.begin(),
                                xbee.rx_frame.end());

            REQUIRE(xbee_payload == payload);

            spi.restoreState();
        }
    }*/
}