/* Copyright (c) 2021 Skyward Experimental Rocketry
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
#include "../catch-tests-entry.cpp"
#endif
#include <ctime>
#include <future>
#include <memory>

#include "MockXbeeSPIBus.h"
#include "drivers/Xbee/Xbee.h"
#include "drivers/spi/SPIDriver.h"
#include <catch2/catch.hpp>

using namespace Xbee;
using std::async;
using std::unique_ptr;

using uint8_ptr = unique_ptr<uint8_t>;

/**
 * @brief Generates a sequence of incremental bytes
 */
uint8_ptr incrementalBytes(size_t length, uint8_t start_from = 0)
{
    uint8_ptr bytes(new uint8_t[length]);

    for (size_t i = 0; i < length; i++)
    {
        bytes.get()[i] = (start_from + i) % 256;
    }

    return bytes;
}

class XbeeWrapper
{
public:
    XbeeWrapper(unsigned int tx_timeout)
        : bus(new MockXbeeSPIBus(xbee_cfg, attn)),
          xbee(new Xbee::Xbee(*bus.get(), xbee_cfg, cs, attn, rst, tx_timeout))
    {
        attn.high();
    }

    MockGpioPin cs;
    MockGpioPin attn;

    MockGpioPin rst;

    SPIBusConfig xbee_cfg{};

    unique_ptr<MockXbeeSPIBus> bus;
    unique_ptr<Xbee::Xbee> xbee;
};

TEST_CASE("[Xbee] Test xbee.send(...) by itself")
{
    uint8_ptr pkt      = incrementalBytes(Xbee::MAX_PACKET_PAYLOAD_LENGTH + 1);
    uint8_ptr pkt_orig = incrementalBytes(Xbee::MAX_PACKET_PAYLOAD_LENGTH + 1);

    XbeeWrapper wrap(DEFAULT_TX_TIMEOUT);

    SECTION("Middle payload length")
    {
        REQUIRE(wrap.xbee->send(pkt.get(), 128));
        REQUIRE(wrap.bus->getParsedFrames().size() > 0);

        REQUIRE(wrap.bus->getParsedFrames().front().frame_type ==
                Xbee::FTYPE_TX_REQUEST);

        TXRequestFrame tx_req = *wrap.bus->getParsedFrames()
                                     .front()
                                     .toFrameType<Xbee::TXRequestFrame>();

        REQUIRE(tx_req.getRFDataLength() == 128);
        REQUIRE(memcmp(tx_req.getRFDataPointer(), pkt_orig.get(), 128) == 0);
    }

    SECTION("Max payload length")
    {
        REQUIRE(wrap.xbee->send(pkt.get(), Xbee::MAX_PACKET_PAYLOAD_LENGTH));
        REQUIRE(wrap.bus->getParsedFrames().size() > 0);

        REQUIRE(wrap.bus->getParsedFrames().front().frame_type ==
                Xbee::FTYPE_TX_REQUEST);

        TXRequestFrame tx_req = *wrap.bus->getParsedFrames()
                                     .front()
                                     .toFrameType<Xbee::TXRequestFrame>();

        REQUIRE(tx_req.getRFDataLength() == Xbee::MAX_PACKET_PAYLOAD_LENGTH);
        REQUIRE(memcmp(tx_req.getRFDataPointer(), pkt_orig.get(),
                       Xbee::MAX_PACKET_PAYLOAD_LENGTH) == 0);
    }

    SECTION("Oversize payload")
    {
        REQUIRE_FALSE(
            wrap.xbee->send(pkt.get(), Xbee::MAX_PACKET_PAYLOAD_LENGTH + 1));
        REQUIRE(wrap.bus->getParsedFrames().size() == 0);
    }

    SECTION("No payload")
    {
        REQUIRE_FALSE(wrap.xbee->send(pkt.get(), 0));
        REQUIRE(wrap.bus->getParsedFrames().size() == 0);
    }

    SECTION("Send error")
    {
        wrap.bus->setRespondWithTxStatus(true, DELS_NO_SPECTRUM_AVAILABLE);

        REQUIRE_FALSE(
            wrap.xbee->send(pkt.get(), Xbee::MAX_PACKET_PAYLOAD_LENGTH));
    }

    SECTION("TX status timeout")
    {
        wrap.bus->setRespondWithTxStatus(false);

        long long start = miosix::getTick();

        REQUIRE_FALSE(
            wrap.xbee->send(pkt.get(), Xbee::MAX_PACKET_PAYLOAD_LENGTH));

        // Should not have returned until the timeout has expired
        REQUIRE(miosix::getTick() >= start + DEFAULT_TX_TIMEOUT);
    }
}

TEST_CASE("[Xbee] Test xbee.receive(...) by itself")
{
    uint8_ptr rx_buf(new uint8_t[MAX_PACKET_PAYLOAD_LENGTH]);

    uint8_ptr pkt = incrementalBytes(Xbee::MAX_PACKET_PAYLOAD_LENGTH);

    XbeeWrapper wrap(DEFAULT_TX_TIMEOUT);

    RXPacketFrame rx;
    rx.setRXDataLength(MAX_PACKET_PAYLOAD_LENGTH);
    memcpy(rx.getRXDataPointer(), pkt.get(), MAX_PACKET_PAYLOAD_LENGTH);
    rx.setReceiveOptions(RO_POINT_MULTIPOINT);
    rx.setSourceAddress(0x1122334455667788);

    SECTION("RX buffer bigger than RX packet payload")
    {
        rx.setRXDataLength(50);
        rx.calcChecksum();

        wrap.bus->pushApiFrame(rx);

        REQUIRE(wrap.xbee->receive(rx_buf.get(), MAX_PACKET_PAYLOAD_LENGTH) ==
                50);

        REQUIRE(memcmp(rx_buf.get(), pkt.get(), 50) == 0);
    }

    SECTION("RX buffer smaller than RX packet payload")
    {
        rx.setRXDataLength(130);
        rx.calcChecksum();

        wrap.bus->pushApiFrame(rx);

        REQUIRE(wrap.xbee->receive(rx_buf.get(), 50) == 50);
        REQUIRE(wrap.xbee->receive(rx_buf.get() + 50, 50) == 50);
        REQUIRE(wrap.xbee->receive(rx_buf.get() + 100, 50) == 30);

        REQUIRE(memcmp(rx_buf.get(), pkt.get(), 130) == 0);
    }

    SECTION("RX buffer == 1")
    {
        rx.setRXDataLength(30);
        rx.calcChecksum();

        wrap.bus->pushApiFrame(rx);

        for (int i = 0; i < 30; i++)
        {
            REQUIRE(wrap.xbee->receive(rx_buf.get() + i, 1) == 1);
        }

        REQUIRE(memcmp(rx_buf.get(), pkt.get(), 30) == 0);
    }

    SECTION("RX packet with wrong checksum")
    {
        rx.setRXDataLength(20);
        rx.calcChecksum();
        wrap.bus->pushApiFrame(rx);

        rx.setRXDataLength(21);
        rx.calcChecksum();
        rx.setRXDataLength(20);  // Now the checksum is wrong
        wrap.bus->pushApiFrame(rx);

        // Only receive one of the two frames
        REQUIRE(wrap.xbee->receive(rx_buf.get(), 50) == 20);

        auto dont_care = async(std::launch::async, [&]() {
            miosix::Thread::sleep(1000);
            wrap.xbee->wakeReceiver(true);
        });

        // This should block until we force it to return
        REQUIRE(wrap.xbee->receive(rx_buf.get(), 50) == -1);
    }

    SECTION("receive() blocks until an interrupt is received")
    {
        rx.setRXDataLength(20);
        rx.calcChecksum();

        // Send an rx frame in one second
        auto dont_care = async(std::launch::async, [&]() {
            miosix::Thread::sleep(1000);
            wrap.bus->pushApiFrame(rx);
            wrap.xbee->wakeReceiver();
        });

        long long start = miosix::getTick();
        REQUIRE(wrap.xbee->receive(rx_buf.get(), 50) == 20);

        // Should not have returned before we sent the rx frame
        REQUIRE(miosix::getTick() >= start + 1000);
    }
}

TEST_CASE("[Xbee] Receive while sending")
{
    uint8_ptr rx_buf(new uint8_t[MAX_PACKET_PAYLOAD_LENGTH]);

    uint8_ptr pkt_rx = incrementalBytes(Xbee::MAX_PACKET_PAYLOAD_LENGTH);
    uint8_ptr pkt_tx = incrementalBytes(Xbee::MAX_PACKET_PAYLOAD_LENGTH);

    XbeeWrapper wrap(1000);
    wrap.bus->setRespondWithTxStatus(true, 0);

    RXPacketFrame rx;
    rx.setRXDataLength(MAX_PACKET_PAYLOAD_LENGTH);
    memcpy(rx.getRXDataPointer(), pkt_rx.get(), MAX_PACKET_PAYLOAD_LENGTH);
    rx.setReceiveOptions(RO_POINT_MULTIPOINT);
    rx.setSourceAddress(0x1122334455667788);

    int num_rx_while_tx = 0;

    wrap.xbee->setOnFrameReceivedListener([&](APIFrame& api) {
        if (api.frame_type == FTYPE_RX_PACKET_FRAME)
        {
            ++num_rx_while_tx;
        }
    });

    SECTION("2 RX packet smaller than one TX packet")
    {
        rx.setRXDataLength(50);
        rx.calcChecksum();

        wrap.bus->pushApiFrame(rx);
        wrap.bus->pushApiFrame(rx);

        REQUIRE(wrap.xbee->send(pkt_tx.get(), 150));

        REQUIRE(num_rx_while_tx == 2);

        size_t rx_len = 0;
        while (rx_len < 100)
        {
            rx_len += wrap.xbee->receive(rx_buf.get() + rx_len, 200);
        }

        REQUIRE(rx_len == 100);
        REQUIRE(memcmp(rx_buf.get(), pkt_rx.get(), 50) == 0);
        REQUIRE(memcmp(rx_buf.get() + 50, pkt_rx.get(), 50) == 0);
    }

    SECTION("More RX packets than buffer size: drop the oldest")
    {
        rx.setRXDataLength(10);

        // 1 packet too many
        unsigned int numpkts = RX_FRAMES_BUF_SIZE + 1;
        for (unsigned int i = 0; i < numpkts; i++)
        {
            memcpy(rx.getRXDataPointer(), pkt_rx.get() + i, 10);
            rx.calcChecksum();
            wrap.bus->pushApiFrame(rx);
        }

        // Should receive all the packets while sending
        REQUIRE(wrap.xbee->send(pkt_tx.get(), MAX_PACKET_PAYLOAD_LENGTH));

        REQUIRE(num_rx_while_tx == numpkts);

        // Only receive the last three packets
        size_t rx_len = 0;
        while (rx_len < (numpkts - 1) * 10)
        {
            rx_len += wrap.xbee->receive(rx_buf.get() + rx_len, 200);
        }

        REQUIRE(rx_len == (numpkts - 1) * 10);

        for (unsigned int i = 0; i < numpkts - 1; i++)
        {
            REQUIRE(memcmp(rx_buf.get() + i * 10, pkt_rx.get() + i + 1, 10) ==
                    0);
        }
    }

    SECTION("2 RX packet smaller than one TX packet, 1 byte receive")
    {
        rx.setRXDataLength(50);
        rx.calcChecksum();

        wrap.bus->pushApiFrame(rx);
        wrap.bus->pushApiFrame(rx);

        REQUIRE(wrap.xbee->send(pkt_tx.get(), 150));

        REQUIRE(num_rx_while_tx == 2);

        for (int i = 0; i < 50; i++)
        {
            REQUIRE(wrap.xbee->receive(rx_buf.get() + i, 1) == 1);
        }
        REQUIRE(memcmp(rx_buf.get(), pkt_rx.get(), 50) == 0);

        for (int i = 0; i < 50; i++)
        {
            REQUIRE(wrap.xbee->receive(rx_buf.get() + i, 1) == 1);
        }
        REQUIRE(memcmp(rx_buf.get(), pkt_rx.get(), 50) == 0);
    }
}

TEST_CASE("[Xbee] Test Xbee::sendAtCommand(...)")
{
    XbeeWrapper wrap(DEFAULT_TX_TIMEOUT);

    SECTION("AT Command, no response required")
    {
        wrap.xbee->sendATCommand("AB");

        REQUIRE(wrap.bus->getParsedFrames().size() > 0);
        REQUIRE(wrap.bus->getParsedFrames()[0].frame_type == FTYPE_AT_COMMAND);
    }

    SECTION("AT Command, response required but not received")
    {
        ATCommandResponseFrame response;
        long long start = miosix::getTick();
        REQUIRE_FALSE(
            wrap.xbee->sendATCommand("AB", &response, nullptr, 0, 1000));
        REQUIRE(miosix::getTick() >= start + 1000);

        REQUIRE(wrap.bus->getParsedFrames().size() > 0);
        REQUIRE(wrap.bus->getParsedFrames()[0].frame_type == FTYPE_AT_COMMAND);
    }

    SECTION("AT Command, response required and received")
    {
        // Padding required in order to not receive the ATCommandResponse
        // frame too early in testing
        RXPacketFrame padding;
        padding.setRXDataLength(50);
        padding.calcChecksum();

        ATCommandResponseFrame resp;
        resp.setATCommand("AB");
        resp.setFrameID(1);
        resp.setCommandDataSize(1);
        resp.getCommandDataPointer()[0] = 0xAB;
        resp.calcChecksum();

        wrap.bus->pushApiFrame(padding);
        wrap.bus->pushApiFrame(resp);

        ATCommandResponseFrame received_response;
        long long start = miosix::getTick();

        REQUIRE(wrap.xbee->sendATCommand("AB", &received_response, nullptr, 0,
                                         1000));
        REQUIRE(miosix::getTick() < start + 1000);

        REQUIRE(wrap.bus->getParsedFrames().size() > 0);
        REQUIRE(wrap.bus->getParsedFrames()[0].frame_type == FTYPE_AT_COMMAND);

        REQUIRE(received_response.getCommandDataLength() == 1);
        REQUIRE(received_response.getCommandDataPointer()[0] == 0xAB);
    }
}